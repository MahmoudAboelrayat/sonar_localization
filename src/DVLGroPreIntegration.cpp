// DVLGroPreIntegration.cpp — minimal implementation of the 6 methods used by
// dvl_imu_gtsam_factors.h.  Math follows ORB-SLAM3 / AQUA-SLAM preintegration
// (Forster et al. 2016) adapted for DVL+gyro+accelerometer.

#include "DVLGroPreIntegration.h"
#include <opencv2/core.hpp>
#include <cmath>

// ── Local SO3 math helpers (float, cv::Mat 3×1 / 3×3) ────────────────────

static cv::Mat skew(const cv::Mat& v)
{
    float x = v.at<float>(0), y = v.at<float>(1), z = v.at<float>(2);
    return (cv::Mat_<float>(3,3) <<
         0, -z,  y,
         z,  0, -x,
        -y,  x,  0);
}

// Rodrigues: v is axis-angle (3×1 float)
static cv::Mat expSO3(const cv::Mat& v)
{
    float th2 = (float)(v.dot(v));
    float th  = std::sqrt(th2);
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    if (th < 1e-7f) return I + skew(v);
    cv::Mat vn = v / th;
    float s = std::sin(th), c = std::cos(th);
    return c * I + (1.0f - c) * vn * vn.t() + s * skew(vn);
}

// Right Jacobian of SO3: Jr(v) = I - (1-cos‖v‖)/‖v‖² [v]× + (‖v‖−sin‖v‖)/‖v‖³ [v]×²
static cv::Mat rightJacSO3(const cv::Mat& v)
{
    float th2 = (float)(v.dot(v));
    float th  = std::sqrt(th2);
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
    if (th < 1e-7f) return I;
    cv::Mat Sv = skew(v);
    return I
         - (1.0f - std::cos(th)) / th2 * Sv
         + (th - std::sin(th)) / (th2 * th) * Sv * Sv;
}

// Project R back onto SO3 via SVD
static cv::Mat normRot(const cv::Mat& R)
{
    cv::Mat U, W, Vt;
    cv::SVDecomp(R, W, U, Vt);
    return U * Vt;
}

// ── Constructor ───────────────────────────────────────────────────────────

DVLGroPreIntegration::DVLGroPreIntegration(
    const Bias& b_, const Calib& calib, bool bDVL_)
    : mb(b_), mCalib(calib), bDVL(bDVL_), dT(0.0), mVelocityThreshold(1e9)
{
    dR      = cv::Mat::eye(3, 3, CV_32F);
    dV      = cv::Mat::zeros(3, 1, CV_32F);
    dP_dvl  = cv::Mat::zeros(3, 1, CV_32F);
    dP_acc  = cv::Mat::zeros(3, 1, CV_32F);
    dDeltaV = cv::Mat::zeros(3, 1, CV_32F);
    mVelocity = cv::Mat::zeros(3, 1, CV_32F);
    mR_g_d  = cv::Mat::eye(3, 3, CV_32F);

    JRg = cv::Mat::zeros(3, 3, CV_32F);
    JVg = cv::Mat::zeros(3, 3, CV_32F);
    JVa = cv::Mat::zeros(3, 3, CV_32F);
    JPg = cv::Mat::zeros(3, 3, CV_32F);
    JPa = cv::Mat::zeros(3, 3, CV_32F);
    JPv = cv::Mat::zeros(3, 3, CV_32F);
    avgA = cv::Mat::zeros(3, 1, CV_32F);
    avgW = cv::Mat::zeros(3, 1, CV_32F);
    C    = cv::Mat::zeros(15, 15, CV_32F);
    Info = cv::Mat::zeros(15, 15, CV_32F);
    Nga  = cv::Mat::zeros(6, 6, CV_32F);
    NgaWalk = cv::Mat::zeros(6, 6, CV_32F);
    db   = cv::Mat::zeros(6, 1, CV_32F);
    bu   = b_;
}

// ── IntegrateGroAccMeasurement ─────────────────────────────────────────────
//
// Integrates one raw IMU sample (accelerometer + gyro) in the sensor frame.
// IMU::Calib applies the extrinsic rotation (mT_gyro_c) internally here,
// so raw sensor-frame measurements are passed from the callback.

void DVLGroPreIntegration::IntegrateGroAccMeasurement(
    const cv::Point3d& acc, const cv::Point3d& angVel, const double& dt)
{
    if (!mCalib.mT_gyro_c.empty()) {
        cv::Mat Rgb = mCalib.mT_gyro_c.rowRange(0, 3).colRange(0, 3);

        cv::Mat a_raw = (cv::Mat_<float>(3, 1) <<
            (float)acc.x, (float)acc.y, (float)acc.z);
        cv::Mat w_raw = (cv::Mat_<float>(3, 1) <<
            (float)angVel.x, (float)angVel.y, (float)angVel.z);

        cv::Mat a_b = Rgb * a_raw;
        cv::Mat w_b = Rgb * w_raw;

        cv::Mat ba = (cv::Mat_<float>(3, 1) <<
            (float)mb.bax, (float)mb.bay, (float)mb.baz);
        cv::Mat bw = (cv::Mat_<float>(3, 1) <<
            (float)mb.bwx, (float)mb.bwy, (float)mb.bwz);

        cv::Mat a_u = a_b - ba;
        cv::Mat w_u = w_b - bw;

        float fdt = (float)dt;
        cv::Mat Sa = skew(a_u);

        // Update position Jacobians first (they depend on the old JVg, JVa, dR)
        JPg += JVg * fdt - 0.5f * dR * Sa * JRg * fdt * fdt;
        JPa += JVa * fdt - 0.5f * dR * fdt * fdt;

        // Update velocity Jacobians
        JVg += -dR * Sa * JRg * fdt;
        JVa += -dR * fdt;

        // Update position (needs old dV)
        dP_acc += dV * fdt + 0.5f * dR * a_u * fdt * fdt;

        // Update velocity
        dV += dR * a_u * fdt;

        // Update rotation Jacobian and rotation
        cv::Mat w_dt = w_u * fdt;
        cv::Mat dR_step = expSO3(w_dt);
        cv::Mat Jr = rightJacSO3(w_dt);
        // dR(b) ≈ dR * ExpSO3(JRg * Δbg); propagation:
        // JRg_new = dR_step^T * JRg - Jr * dt
        JRg = dR_step.t() * JRg - Jr * fdt;

        dR = normRot(dR * dR_step);
        dT += (double)fdt;

        // Running averages (informational)
        avgA += a_u * fdt;
        avgW += w_u * fdt;

        mAngV = angVel;  // keep last angular velocity for DVL integration
    }
}

// ── IntegrateDVLMeasurement ────────────────────────────────────────────────
//
// Integrates one DVL velocity measurement into dP_dvl.
// v_dk is in the DVL sensor frame; we rotate to body frame via mT_dvl_c.

void DVLGroPreIntegration::IntegrateDVLMeasurement(
    const cv::Point3d& v_dk, const double& dt)
{
    cv::Mat v_raw = (cv::Mat_<float>(3, 1) <<
        (float)v_dk.x, (float)v_dk.y, (float)v_dk.z);

    // Rotate DVL velocity to body frame if extrinsic is available
    cv::Mat v_body = v_raw;
    if (!mCalib.mT_dvl_c.empty()) {
        cv::Mat R_db = mCalib.mT_dvl_c.rowRange(0, 3).colRange(0, 3);  // DVL→body
        v_body = R_db * v_raw;
    }

    dP_dvl += v_body * (float)dt;
    mVelocity = v_body;

    v_dk_dvl = cv::Point3d(v_body.at<float>(0),
                           v_body.at<float>(1),
                           v_body.at<float>(2));
}

// ── GetDeltaRotation ──────────────────────────────────────────────────────
//
// First-order bias correction: dR(b) ≈ dR * ExpSO3(JRg * Δbg)

cv::Mat DVLGroPreIntegration::GetDeltaRotation(const Bias& b_)
{
    cv::Mat dbg = (cv::Mat_<float>(3, 1) <<
        (float)(b_.bwx - mb.bwx),
        (float)(b_.bwy - mb.bwy),
        (float)(b_.bwz - mb.bwz));
    return normRot(dR * expSO3(JRg * dbg));
}

// ── GetDeltaVelocity ──────────────────────────────────────────────────────
//
// First-order bias correction: dV(b) = dV + JVg*Δbg + JVa*Δba

cv::Mat DVLGroPreIntegration::GetDeltaVelocity(const Bias& b_)
{
    cv::Mat dbg = (cv::Mat_<float>(3, 1) <<
        (float)(b_.bwx - mb.bwx),
        (float)(b_.bwy - mb.bwy),
        (float)(b_.bwz - mb.bwz));
    cv::Mat dba = (cv::Mat_<float>(3, 1) <<
        (float)(b_.bax - mb.bax),
        (float)(b_.bay - mb.bay),
        (float)(b_.baz - mb.baz));
    return dV + JVg * dbg + JVa * dba;
}

// ── GetDeltaPosition ──────────────────────────────────────────────────────
//
// Returns the preintegrated position change.
// When bDVL=false: accelerometer-based integral with bias correction.
// When bDVL=true:  DVL-based integral (no bias correction — DVL is a direct
//                  velocity measurement, not sensitive to IMU bias).

cv::Mat DVLGroPreIntegration::GetDeltaPosition(const Bias& b_)
{
    if (bDVL)
        return dP_dvl;

    cv::Mat dbg = (cv::Mat_<float>(3, 1) <<
        (float)(b_.bwx - mb.bwx),
        (float)(b_.bwy - mb.bwy),
        (float)(b_.bwz - mb.bwz));
    cv::Mat dba = (cv::Mat_<float>(3, 1) <<
        (float)(b_.bax - mb.bax),
        (float)(b_.bay - mb.bay),
        (float)(b_.baz - mb.baz));
    return dP_acc + JPg * dbg + JPa * dba;
}
