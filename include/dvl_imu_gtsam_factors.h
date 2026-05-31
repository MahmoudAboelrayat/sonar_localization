/**
 * GTSAM port of AQUA-SLAM DVL-IMU g2o factors
 *
 * Ported directly from:
 *   EdgeDvlIMUInitWithoutBias  → DvlImuInitFactor        (gravity init, no bias)
 *   EdgeDvlIMUInit             → DvlImuInitFactor        (same residual, different prior)
 *   EdgeDvlIMUGravityRefine    → DvlImuGravityRefineFactor
 *   EdgeDvlIMUGravityRefineWithBias → DvlImuGravityRefineWithBiasFactor
 *   EdgeDvlIMU                 → DvlImuFactor            (full tightly-coupled factor)
 *
 * Variable keys used:
 *   X(i)  — Pose3  (camera/body pose at keyframe i)
 *   V(i)  — Vector3 (DVL velocity in world frame at keyframe i)
 *   B(0)  — imuBias::ConstantBias (shared gyro + accel bias)
 *   T_DC  — Pose3  (T_dvl_camera extrinsic, optimizable)
 *   T_GD  — Pose3  (T_gyro_dvl extrinsic, optimizable)
 *   G(0)  — Vector2 (gravity direction: roll + pitch, parameterized as Rwg)
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/Manifold.h>
#include <boost/optional.hpp>
#include <functional>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ORB-SLAM3 preintegration (keep as-is, no need to rewrite)
#include "ImuTypes.h"
#include "DVLGroPreIntegration.h"

using namespace ORB_SLAM3;

// ─────────────────────────────────────────────────────────────────────────────
// Gravity direction helper — mirrors ORB-SLAM3 GDirection / VertexGDir
//
// Gravity is stored as a full 3x3 rotation Rwg that maps canonical gravity
// [0,0,-9.81] into the world frame. Only 2 DOF (roll + pitch) are observable.
// We store Rwg as a Matrix3d and update it via Exp(delta_roll, delta_pitch, 0).
// ─────────────────────────────────────────────────────────────────────────────
struct GravityDir
{
    Eigen::Matrix3d Rwg;  // world ← gravity frame rotation

    GravityDir() : Rwg(Eigen::Matrix3d::Identity()) {}
    explicit GravityDir(const Eigen::Matrix3d& R) : Rwg(R) {}

    // Apply a 2-DOF perturbation [delta_roll, delta_pitch]
    GravityDir retract(const Eigen::Vector2d& delta) const
    {
        // Only roll and pitch — yaw is unobservable
        Eigen::AngleAxisd roll (delta(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(delta(1), Eigen::Vector3d::UnitY());
        return GravityDir(Rwg * (roll * pitch).toRotationMatrix());
    }

    // Gravity vector in world frame
    Eigen::Vector3d gravityWorld() const
    {
        static const Eigen::Vector3d gI(0, 0, -9.81);
        return Rwg * gI;
    }
};

// ── GTSAM traits for GravityDir (2-DOF roll/pitch manifold) ─────────────────
namespace gtsam {
template<>
struct traits<GravityDir> {
    typedef manifold_tag structure_category;
    typedef GravityDir   ManifoldType;
    enum { dimension = 2 };

    static void Print(const GravityDir& g, const std::string& s = "") {
        std::cout << s << "GravityDir Rwg:\n" << g.Rwg << std::endl;
    }
    static bool Equals(const GravityDir& a, const GravityDir& b, double tol = 1e-8) {
        return (a.Rwg - b.Rwg).norm() < tol;
    }
    static int GetDimension(const GravityDir&) { return dimension; }
    static GravityDir Retract(const GravityDir& g, const Vector& xi) {
        return g.retract(xi.head<2>());
    }
    static Vector Local(const GravityDir& g, const GravityDir& h) {
        Eigen::Matrix3d dR = g.Rwg.transpose() * h.Rwg;
        Eigen::AngleAxisd aa(dR);
        Vector2 out;
        out << aa.angle() * aa.axis()(0), aa.angle() * aa.axis()(1);
        return out;
    }
};
} // namespace gtsam

// ── Numerical Jacobian helpers ────────────────────────────────────────────────
// J = df/dx_k  (error_dim × tangent_dim). e0 = f(x) pre-computed.
template<typename T>
static inline gtsam::Matrix numDeriv(
    const gtsam::Values& x, gtsam::Key k,
    const std::function<gtsam::Vector(const gtsam::Values&)>& f,
    const gtsam::Vector& e0, double eps = 1e-5)
{
    const T& v0  = x.at<T>(k);
    int      dim = gtsam::traits<T>::GetDimension(v0);
    gtsam::Matrix J(e0.size(), dim);
    for (int j = 0; j < dim; ++j) {
        gtsam::Vector xi = gtsam::Vector::Zero(dim);
        xi(j) = eps;
        gtsam::Values xp = x;
        xp.update(k, gtsam::traits<T>::Retract(v0, xi));
        J.col(j) = (f(xp) - e0) / eps;
    }
    return J;
}

// Fill a 9-key Jacobian vector with types matching the DVL-IMU factor key order:
// [Pose3, Pose3, Vector3, Vector3, Vector3, Vector3, Pose3, Pose3, GravityDir]
static inline void fillJacobians9(
    const gtsam::Values& x, const gtsam::KeyVector& ks,
    const std::function<gtsam::Vector(const gtsam::Values&)>& f,
    const gtsam::Vector& e0,
    boost::optional<std::vector<gtsam::Matrix>&> H)
{
    if (!H) return;
    H->resize(9);
    (*H)[0] = numDeriv<gtsam::Pose3  >(x, ks[0], f, e0);
    (*H)[1] = numDeriv<gtsam::Pose3  >(x, ks[1], f, e0);
    (*H)[2] = numDeriv<gtsam::Vector3>(x, ks[2], f, e0);
    (*H)[3] = numDeriv<gtsam::Vector3>(x, ks[3], f, e0);
    (*H)[4] = numDeriv<gtsam::Vector3>(x, ks[4], f, e0);
    (*H)[5] = numDeriv<gtsam::Vector3>(x, ks[5], f, e0);
    (*H)[6] = numDeriv<gtsam::Pose3  >(x, ks[6], f, e0);
    (*H)[7] = numDeriv<gtsam::Pose3  >(x, ks[7], f, e0);
    (*H)[8] = numDeriv<GravityDir    >(x, ks[8], f, e0);
}

// SO3 log map (same as ORB-SLAM3 LogSO3)
inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: build IMU::Bias from a gtsam::imuBias::ConstantBias
// ─────────────────────────────────────────────────────────────────────────────
inline IMU::Bias toOrbBias(const gtsam::imuBias::ConstantBias& b)
{
    auto ba = b.accelerometer();
    auto bg = b.gyroscope();
    return IMU::Bias(ba(0), ba(1), ba(2), bg(0), bg(1), bg(2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: cv::Mat 3x1 → Eigen::Vector3d
// ─────────────────────────────────────────────────────────────────────────────
inline Eigen::Vector3d toVec3(const cv::Mat& m)
{
    return Eigen::Vector3d(m.at<float>(0), m.at<float>(1), m.at<float>(2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: cv::Mat 3x3 → Eigen::Matrix3d
// ─────────────────────────────────────────────────────────────────────────────
inline Eigen::Matrix3d toMat3(const cv::Mat& m)
{
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = m.at<float>(i, j);
    return R;
}


// =============================================================================
// FACTOR 1: DvlImuInitFactor
//
// Ports: EdgeDvlIMUInitWithoutBias and EdgeDvlIMUInit
//
// Used during initialization. Residual is velocity-only (3D).
// Compares DVL-observed velocity change against gyro-preintegrated dV.
//
// Variables: pose_i, pose_j, vel_i, vel_j, gyro_bias, accel_bias,
//            T_dvl_cam, T_gyro_dvl, gravity_dir
//
// Residual (3D):
//   VDelta_est = R_gyro_dvl * R_dvl_cam * R_ci_w *
//                (R_w_cj * R_cam_dvl * v2
//               - R_w_ci * R_cam_dvl * v1
//               - R_cam_dvl * R_dvl_gyro * Rwg * g * dt)
//   e_V = VDelta_est - dV(bias)
// =============================================================================
class DvlImuInitFactor : public gtsam::NoiseModelFactor
{
public:
    // Keys order: pose_i, pose_j, vel_i, vel_j, gyro_bias, accel_bias,
    //             T_dvl_cam, T_gyro_dvl, gravity_dir
    DvlImuInitFactor(
        gtsam::Key pose_i, gtsam::Key pose_j,
        gtsam::Key vel_i,  gtsam::Key vel_j,
        gtsam::Key gyro_bias, gtsam::Key accel_bias,
        gtsam::Key T_dvl_cam, gtsam::Key T_gyro_dvl,
        gtsam::Key gravity_dir,
        DVLGroPreIntegration* pim,
        const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor(noise,
              gtsam::KeyVector{pose_i, pose_j, vel_i, vel_j,
                               gyro_bias, accel_bias, T_dvl_cam, T_gyro_dvl, gravity_dir})
        , pim_(pim)
    {}

    gtsam::Vector unwhitenedError(
        const gtsam::Values& x,
        boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override
    {
        // Retrieve variables
        auto pose_i   = x.at<gtsam::Pose3>(keys()[0]);
        auto pose_j   = x.at<gtsam::Pose3>(keys()[1]);
        auto v1       = x.at<gtsam::Vector3>(keys()[2]);
        auto v2       = x.at<gtsam::Vector3>(keys()[3]);
        auto bg       = x.at<gtsam::Vector3>(keys()[4]);
        auto ba       = x.at<gtsam::Vector3>(keys()[5]);
        auto T_dvl_cam  = x.at<gtsam::Pose3>(keys()[6]);
        auto T_gyro_dvl = x.at<gtsam::Pose3>(keys()[7]);
        auto gdir     = x.at<GravityDir>(keys()[8]);

        // Extract rotations
        Eigen::Matrix3d R_dvl_cam  = T_dvl_cam.rotation().matrix();
        Eigen::Matrix3d R_cam_dvl  = R_dvl_cam.transpose();
        Eigen::Matrix3d R_gyro_dvl = T_gyro_dvl.rotation().matrix();
        Eigen::Matrix3d R_dvl_gyro = R_gyro_dvl.transpose();

        Eigen::Matrix3d R_ci_w = pose_i.rotation().matrix().transpose(); // Rcw
        Eigen::Matrix3d R_w_ci = pose_i.rotation().matrix();             // Rwc
        Eigen::Matrix3d R_w_cj = pose_j.rotation().matrix();             // Rwc

        Eigen::Matrix3d Rwg = gdir.Rwg;
        Eigen::Vector3d g_w = gdir.gravityWorld();

        // Build ORB bias
        IMU::Bias b(ba(0), ba(1), ba(2), bg(0), bg(1), bg(2));
        Eigen::Vector3d dV = toVec3(pim_->GetDeltaVelocity(b));
        double dt = pim_->dT;

        // Mirror of g2o computeError:
        // VDelta_est = R_gyro_dvl * R_dvl_cam * Rci_w *
        //              (Rw_cj * R_cam_dvl * v2
        //             - Rw_ci * R_cam_dvl * v1
        //             - R_cam_dvl * R_dvl_gyro * Rwg * g * dt)
        Eigen::Vector3d VDelta_est =
            R_gyro_dvl * R_dvl_cam * R_ci_w *
            (R_w_cj * R_cam_dvl * v2
           - R_w_ci * R_cam_dvl * v1
           - R_cam_dvl * R_dvl_gyro * g_w * dt);

        gtsam::Vector e_V = VDelta_est - dV;

        if (H) {
            auto f = [this](const gtsam::Values& xp) -> gtsam::Vector {
                return unwhitenedError(xp, boost::none);
            };
            fillJacobians9(x, keys(), f, e_V, H);
        }
        return e_V;
    }

private:
    DVLGroPreIntegration* pim_;
};


// =============================================================================
// FACTOR 2: DvlImuGravityRefineWithBiasFactor
//
// Ports: EdgeDvlIMUGravityRefineWithBias
//
// Full 9D residual: rotation + velocity + position.
// Used after initialization to keep refining gravity, extrinsics, and biases.
//
// Residual (9D):
//   e_R = Log( dR(b)^T * R_b_c * Rci_w * Rw_cj * R_c_b )
//   e_V = R_b_c * Rci_w * (Rw_cj*R_c_dvl*v2 - Rw_ci*R_c_dvl*v1 - R_c_b*Rwg*g*dt) - dV(b)
//   e_P = R_b_c * Rci_w * (Rw_cj*t_c_b + t_w_cj - Rw_ci*t_c_b - t_w_ci
//                          - Rw_ci*R_c_dvl*v1*dt - 0.5*R_c_b*Rwg*g*dt²) - dP(b)
// =============================================================================
class DvlImuGravityRefineWithBiasFactor : public gtsam::NoiseModelFactor
{
public:
    DvlImuGravityRefineWithBiasFactor(
        gtsam::Key pose_i, gtsam::Key pose_j,
        gtsam::Key vel_i,  gtsam::Key vel_j,
        gtsam::Key gyro_bias, gtsam::Key accel_bias,
        gtsam::Key T_dvl_cam, gtsam::Key T_gyro_dvl,
        gtsam::Key gravity_dir,
        DVLGroPreIntegration* pim,
        const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor(noise,
              gtsam::KeyVector{pose_i, pose_j, vel_i, vel_j,
                               gyro_bias, accel_bias, T_dvl_cam, T_gyro_dvl, gravity_dir})
        , pim_(pim), dt_(pim->dT)
    {}

    gtsam::Vector unwhitenedError(
        const gtsam::Values& x,
        boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override
    {
        auto pose_i     = x.at<gtsam::Pose3>(keys()[0]);
        auto pose_j     = x.at<gtsam::Pose3>(keys()[1]);
        auto v1         = x.at<gtsam::Vector3>(keys()[2]);
        auto v2         = x.at<gtsam::Vector3>(keys()[3]);
        auto bg         = x.at<gtsam::Vector3>(keys()[4]);
        auto ba         = x.at<gtsam::Vector3>(keys()[5]);
        auto T_dvl_cam  = x.at<gtsam::Pose3>(keys()[6]);
        auto T_gyro_dvl = x.at<gtsam::Pose3>(keys()[7]);
        auto gdir       = x.at<GravityDir>(keys()[8]);

        Eigen::Matrix3d R_dvl_cam  = T_dvl_cam.rotation().matrix();
        Eigen::Matrix3d R_cam_dvl  = R_dvl_cam.transpose();
        Eigen::Matrix3d R_gyro_dvl = T_gyro_dvl.rotation().matrix();

        // T_b_c = T_gyro_dvl * T_dvl_cam  (gyro frame = body frame)
        gtsam::Pose3 T_b_c = T_gyro_dvl.compose(T_dvl_cam);
        Eigen::Matrix3d R_b_c  = T_b_c.rotation().matrix();
        Eigen::Matrix3d R_c_b  = R_b_c.transpose();
        Eigen::Vector3d t_c_b  = T_b_c.inverse().translation();

        Eigen::Matrix3d R_ci_w = pose_i.rotation().matrix().transpose();
        Eigen::Matrix3d R_w_ci = pose_i.rotation().matrix();
        Eigen::Matrix3d R_w_cj = pose_j.rotation().matrix();
        Eigen::Vector3d t_w_ci = pose_i.translation();
        Eigen::Vector3d t_w_cj = pose_j.translation();

        Eigen::Vector3d g_w = gdir.gravityWorld();

        IMU::Bias b(ba(0), ba(1), ba(2), bg(0), bg(1), bg(2));
        Eigen::Matrix3d dR  = toMat3(pim_->GetDeltaRotation(b));
        Eigen::Vector3d dV  = toVec3(pim_->GetDeltaVelocity(b));
        Eigen::Vector3d dP  = toVec3(pim_->GetDeltaPosition(b));

        // Rotation residual
        Eigen::Matrix3d R_est = R_b_c * R_ci_w * R_w_cj * R_c_b;
        Eigen::Vector3d e_R   = LogSO3(dR.transpose() * R_est);

        // Velocity residual
        Eigen::Vector3d VDelta_est =
            R_b_c * R_ci_w *
            (R_w_cj * R_cam_dvl * v2
           - R_w_ci * R_cam_dvl * v1
           - R_c_b * g_w * dt_);
        Eigen::Vector3d e_V = VDelta_est - dV;

        // Position residual
        Eigen::Vector3d P_acc_est =
            R_b_c * R_ci_w *
            (R_w_cj * t_c_b + t_w_cj
           - (R_w_ci * t_c_b + t_w_ci)
           - R_w_ci * R_cam_dvl * v1 * dt_
           - 0.5 * R_c_b * g_w * dt_ * dt_);
        Eigen::Vector3d e_P = P_acc_est - dP;

        gtsam::Vector9 error;
        error << e_R, e_V, e_P;

        if (H) {
            auto f = [this](const gtsam::Values& xp) -> gtsam::Vector {
                return unwhitenedError(xp, boost::none);
            };
            fillJacobians9(x, keys(), f, error, H);
        }
        return error;
    }

private:
    DVLGroPreIntegration* pim_;
    double dt_;
};


// =============================================================================
// FACTOR 3: DvlImuFactor
//
// Ports: EdgeDvlIMU  — the main tightly-coupled DVL-IMU factor
//
// Full 9D residual using both DVL-position and accelerometer-position.
// This is the factor added for every keyframe pair during tracking.
//
// Same residual as DvlImuGravityRefineWithBiasFactor but also includes
// the DVL-frame position term dP_dvl for the lever arm correction.
// =============================================================================
class DvlImuFactor : public gtsam::NoiseModelFactor
{
public:
    DvlImuFactor(
        gtsam::Key pose_i, gtsam::Key pose_j,
        gtsam::Key vel_i,  gtsam::Key vel_j,
        gtsam::Key gyro_bias, gtsam::Key accel_bias,
        gtsam::Key T_dvl_cam, gtsam::Key T_gyro_dvl,
        gtsam::Key gravity_dir,
        DVLGroPreIntegration* pim,
        const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor(noise,
              gtsam::KeyVector{pose_i, pose_j, vel_i, vel_j,
                               gyro_bias, accel_bias, T_dvl_cam, T_gyro_dvl, gravity_dir})
        , pim_(pim), dt_(pim->dT)
    {}

    gtsam::Vector unwhitenedError(
        const gtsam::Values& x,
        boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override
    {
        auto pose_i     = x.at<gtsam::Pose3>(keys()[0]);
        auto pose_j     = x.at<gtsam::Pose3>(keys()[1]);
        auto v1         = x.at<gtsam::Vector3>(keys()[2]);
        auto v2         = x.at<gtsam::Vector3>(keys()[3]);
        auto bg         = x.at<gtsam::Vector3>(keys()[4]);
        auto ba         = x.at<gtsam::Vector3>(keys()[5]);
        auto T_dvl_cam  = x.at<gtsam::Pose3>(keys()[6]);
        auto T_gyro_dvl = x.at<gtsam::Pose3>(keys()[7]);
        auto gdir       = x.at<GravityDir>(keys()[8]);

        Eigen::Matrix3d R_dvl_cam  = T_dvl_cam.rotation().matrix();
        Eigen::Matrix3d R_cam_dvl  = R_dvl_cam.transpose();
        Eigen::Vector3d t_dvl_cam  = T_dvl_cam.translation();

        gtsam::Pose3 T_b_c = T_gyro_dvl.compose(T_dvl_cam);
        Eigen::Matrix3d R_b_c  = T_b_c.rotation().matrix();
        Eigen::Matrix3d R_c_b  = R_b_c.transpose();
        Eigen::Vector3d t_c_b  = T_b_c.inverse().translation();

        Eigen::Matrix3d R_ci_w = pose_i.rotation().matrix().transpose();
        Eigen::Matrix3d R_w_ci = pose_i.rotation().matrix();
        Eigen::Matrix3d R_w_cj = pose_j.rotation().matrix();
        Eigen::Vector3d t_w_ci = pose_i.translation();
        Eigen::Vector3d t_w_cj = pose_j.translation();

        // Average DVL velocity (midpoint integration)
        Eigen::Vector3d avg_v = 0.5 * (v1 + v2);
        Eigen::Vector3d g_w   = gdir.gravityWorld();

        IMU::Bias b(ba(0), ba(1), ba(2), bg(0), bg(1), bg(2));
        Eigen::Matrix3d dR  = toMat3(pim_->GetDeltaRotation(b));
        Eigen::Vector3d dV  = toVec3(pim_->GetDeltaVelocity(b));
        Eigen::Vector3d dP  = toVec3(pim_->GetDeltaPosition(b));

        // Rotation residual
        Eigen::Matrix3d R_est = R_b_c * R_ci_w * R_w_cj * R_c_b;
        Eigen::Vector3d e_R   = LogSO3(dR.transpose() * R_est);

        // Velocity residual
        Eigen::Vector3d VDelta_est =
            R_b_c * R_ci_w *
            (R_w_cj * R_cam_dvl * v2
           - R_w_ci * R_cam_dvl * v1
           - R_c_b * g_w * dt_);
        Eigen::Vector3d e_V = VDelta_est - dV;

        // Position residual (accelerometer side — from preintegration)
        Eigen::Vector3d P_acc_est =
            R_b_c * R_ci_w *
            (R_w_cj * t_c_b + t_w_cj
           - (R_w_ci * t_c_b + t_w_ci)
           - R_w_ci * R_cam_dvl * v1 * dt_
           - 0.5 * R_c_b * g_w * dt_ * dt_);
        Eigen::Vector3d e_P = P_acc_est - dP;

        gtsam::Vector9 error;
        error << e_R, e_V, e_P;

        if (H) {
            auto f = [this](const gtsam::Values& xp) -> gtsam::Vector {
                return unwhitenedError(xp, boost::none);
            };
            fillJacobians9(x, keys(), f, error, H);
        }
        return error;
    }

private:
    DVLGroPreIntegration* pim_;
    double dt_;
};


// =============================================================================
// HOW TO USE IN YOUR ISAM2 GRAPH
// =============================================================================
//
// ── Setup ────────────────────────────────────────────────────────────────────
//
//   gtsam::ISAM2 isam(gtsam::ISAM2Params());
//   gtsam::NonlinearFactorGraph graph;
//   gtsam::Values initial;
//
//   // Fixed extrinsics (or make them optimizable if you want online calib)
//   gtsam::Pose3 T_dvl_cam  = ...;  // from kalibr
//   gtsam::Pose3 T_gyro_dvl = ...;  // from kalibr
//   GravityDir   gdir;              // starts as identity → seeded before first call
//
//   initial.insert(gtsam::Symbol('E', 0), T_dvl_cam);
//   initial.insert(gtsam::Symbol('F', 0), T_gyro_dvl);
//   initial.insert(gtsam::Symbol('G', 0), gdir);
//
// ── Phase 1 — initialization (not enough motion) ────────────────────────────
//
//   // Hard priors suppress IMU accel bias
//   auto bias_noise_tight = gtsam::noiseModel::Diagonal::Sigmas(
//       (gtsam::Vector6() << 1e-1, 1e-1, 1e-1,    // gyro bias sigma
//                            1e-8, 1e-8, 1e-8)     // accel bias sigma (very tight)
//       .finished());
//   graph.addPrior(B(0), gtsam::imuBias::ConstantBias(), bias_noise_tight);
//
//   graph.emplace_shared<DvlImuInitFactor>(
//       X(i), X(j), V(i), V(j),
//       gtsam::Symbol('b', 0), gtsam::Symbol('a', 0),   // gyro/accel bias keys
//       gtsam::Symbol('E', 0), gtsam::Symbol('F', 0),   // extrinsic keys
//       gtsam::Symbol('G', 0),                           // gravity key
//       pim,
//       gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
//
// ── Phase 2 — enough motion, full factor ────────────────────────────────────
//
//   // Looser priors — let IMU accel contribute
//   auto bias_noise_loose = gtsam::noiseModel::Diagonal::Sigmas(
//       (gtsam::Vector6() << 1e-2, 1e-2, 1e-2,    // gyro bias sigma
//                            1e-3, 1e-3, 1e-3)     // accel bias sigma (loose)
//       .finished());
//
//   graph.emplace_shared<DvlImuFactor>(
//       X(i), X(j), V(i), V(j),
//       gtsam::Symbol('b', 0), gtsam::Symbol('a', 0),
//       gtsam::Symbol('E', 0), gtsam::Symbol('F', 0),
//       gtsam::Symbol('G', 0),
//       pim,
//       gtsam::noiseModel::Isotropic::Sigma(9, 0.05));
//
// ── Gravity refinement after initialization ──────────────────────────────────
//
//   graph.emplace_shared<DvlImuGravityRefineWithBiasFactor>(
//       X(i), X(j), V(i), V(j),
//       gtsam::Symbol('b', 0), gtsam::Symbol('a', 0),
//       gtsam::Symbol('E', 0), gtsam::Symbol('F', 0),
//       gtsam::Symbol('G', 0),
//       pim,
//       gtsam::noiseModel::Isotropic::Sigma(9, 0.02));
//
// ── iSAM2 update ─────────────────────────────────────────────────────────────
//
//   isam.update(graph, initial);
//   auto result = isam.calculateEstimate();
//   gdir = result.at<GravityDir>(gtsam::Symbol('G', 0));
//   graph.resize(0);
//   initial.clear();
