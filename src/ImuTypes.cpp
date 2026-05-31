// ImuTypes.cpp — implementations of the non-inline ImuTypes methods
// that are referenced by the localization_imu_pre build target.

#include "ImuTypes.h"
#include <opencv2/core.hpp>

namespace ORB_SLAM3 {
namespace IMU {

// ── Bias::CopyFrom ────────────────────────────────────────────────────────
void Bias::CopyFrom(Bias& b)
{
    bax = b.bax; bay = b.bay; baz = b.baz;
    bwx = b.bwx; bwy = b.bwy; bwz = b.bwz;
}

// ── Calib::Set ────────────────────────────────────────────────────────────
// Initialises the noise covariance matrices from scalar noise densities.
// Called by the (T_gyro_c, T_dvl_c, ng, na, ngw, naw) constructor.
void Calib::Set(const cv::Mat& Tbc_,
                const float& ng, const float& na,
                const float& ngw, const float& naw)
{
    // Store the gyro-to-body transform (re-uses Tbc / Tcb fields)
    Tbc = Tbc_.clone();
    Tcb = cv::Mat::eye(4, 4, CV_32F);
    Tcb.rowRange(0, 3).colRange(0, 3) =
        Tbc_.rowRange(0, 3).colRange(0, 3).t();
    Tcb.rowRange(0, 3).col(3) =
        -Tbc_.rowRange(0, 3).colRange(0, 3).t()
         * Tbc_.rowRange(0, 3).col(3);

    // Measurement noise covariance [gyro (3×3), accel (3×3)]
    Cov = cv::Mat::eye(6, 6, CV_32F);
    Cov.at<float>(0, 0) = ng  * ng;
    Cov.at<float>(1, 1) = ng  * ng;
    Cov.at<float>(2, 2) = ng  * ng;
    Cov.at<float>(3, 3) = na  * na;
    Cov.at<float>(4, 4) = na  * na;
    Cov.at<float>(5, 5) = na  * na;

    // Bias random-walk noise covariance
    CovWalk = cv::Mat::eye(6, 6, CV_32F);
    CovWalk.at<float>(0, 0) = ngw * ngw;
    CovWalk.at<float>(1, 1) = ngw * ngw;
    CovWalk.at<float>(2, 2) = ngw * ngw;
    CovWalk.at<float>(3, 3) = naw * naw;
    CovWalk.at<float>(4, 4) = naw * naw;
    CovWalk.at<float>(5, 5) = naw * naw;
}

// ── Calib::SetExtrinsic ───────────────────────────────────────────────────
void Calib::SetExtrinsic(const cv::Mat& T_gyro_c, const cv::Mat& T_dvl_c)
{
    mT_gyro_c = T_gyro_c.clone();
    mT_dvl_c  = T_dvl_c.clone();

    mT_c_gyro = cv::Mat::eye(4, 4, CV_32F);
    mT_c_gyro.rowRange(0, 3).colRange(0, 3) =
        mT_gyro_c.rowRange(0, 3).colRange(0, 3).t();
    mT_c_gyro.rowRange(0, 3).col(3) =
        -mT_gyro_c.rowRange(0, 3).colRange(0, 3).t()
         * mT_gyro_c.rowRange(0, 3).col(3);

    mT_c_dvl = cv::Mat::eye(4, 4, CV_32F);
    mT_c_dvl.rowRange(0, 3).colRange(0, 3) =
        mT_dvl_c.rowRange(0, 3).colRange(0, 3).t();
    mT_c_dvl.rowRange(0, 3).col(3) =
        -mT_dvl_c.rowRange(0, 3).colRange(0, 3).t()
         * mT_dvl_c.rowRange(0, 3).col(3);

    mT_gyro_dvl = mT_gyro_c * mT_c_dvl;
}

} // namespace IMU
} // namespace ORB_SLAM3
