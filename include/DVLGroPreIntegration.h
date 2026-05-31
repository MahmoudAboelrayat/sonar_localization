//
// Created by da on 25/01/2021.
//

#ifndef DVLGROPREINTEGRATION_H
#define DVLGROPREINTEGRATION_H

#include <ImuTypes.h>

using namespace ORB_SLAM3;
using namespace IMU;

class DVLGroPreIntegration
{


    template<typename MatrixType, typename Archive>
    void serializeEigenMatrix(Archive &ar, MatrixType &m, const unsigned int version)
    {
        int cols = m.cols();
        int rows = m.rows();
        ar & cols;
        ar & rows;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                ar & m(i, j);
            }
        }
    }

	template<class Archive>
	void serializePoint3f(Archive &ar, cv::Point3f &p, const unsigned int version)
	{
		float x, y, z;
		if (Archive::is_saving::value) {
			x = p.x;
			y = p.y;
			z = p.z;
			ar & x;
			ar & y;
			ar & z;
		}
		else if (Archive::is_loading::value) {
			ar & x;
			ar & y;
			ar & z;
			p = cv::Point3f(x, y, z);
		}
	}

	template<class Archive>
	void serializePoint3d(Archive &ar, cv::Point3d &p, const unsigned int version)
	{
		double x, y, z;
		if (Archive::is_saving::value) {
			x = p.x;
			y = p.y;
			z = p.z;
			ar & x;
			ar & y;
			ar & z;
		}
		else if (Archive::is_loading::value) {
			ar & x;
			ar & y;
			ar & z;
			p = cv::Point3d(x, y, z);
		}
	}

	template<class Archive>
	void serializeMatrix(Archive &ar, cv::Mat &mat, const unsigned int version)
	{
		int cols, rows, type;
		bool continuous;

		if (Archive::is_saving::value) {
			cols = mat.cols;
			rows = mat.rows;
			type = mat.type();
			continuous = mat.isContinuous();
		}

		ar & cols & rows & type & continuous;
		if (Archive::is_loading::value) {
			mat.create(rows, cols, type);
		}

		if (continuous) {
			const unsigned int data_size = rows * cols * mat.elemSize();
			ar & boost::serialization::make_array(mat.ptr(), data_size);
		}
		else {
			const unsigned int row_size = cols * mat.elemSize();
			for (int i = 0; i < rows; i++) {
				ar & boost::serialization::make_array(mat.ptr(i), row_size);
			}
		}
	}

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version);
public:
	DVLGroPreIntegration(const Bias &b_, const Calib &calib, bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_, const Calib &calib, const cv::Point3d &v_di, bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_,
	                     const Calib &calib,
	                     const cv::Point3d &v_di,
	                     const Eigen::Vector4d &alpha,
	                     const Eigen::Vector4d &beta,
	                     bool bDVL = false);
	DVLGroPreIntegration(const Bias &b_,
	                     const Calib &calib,
	                     const cv::Point3d &v_di,
	                     double velocity_threshold,
	                     bool bDVL = false);
	DVLGroPreIntegration(DVLGroPreIntegration *pDVLPre);
    DVLGroPreIntegration(const DVLGroPreIntegration& integration);
	DVLGroPreIntegration()
	{}
	~DVLGroPreIntegration()
	{}
	void Initialize(const Bias &b_);

	//  first Integrate Gro, then DVL
	void IntegrateGroMeasurement(const cv::Point3d &angVel, const double &dt);

	/***
	 * integrate gyro and acc
	 * @param acc acceloration
	 * @param angVel angular velocity
	 * @param dt duration
	 */
	void IntegrateGroAccMeasurement(const cv::Point3d &acc, const cv::Point3d &angVel, const double &dt);

//	void IntegrateGroMeasurement2(const cv::Point3d &angVel, const double &dt);

	/*!
	 *
	 * @param v_dk: V_dk
	 */
	void IntegrateDVLMeasurement(const cv::Point3d &v_dk, const double &dt);

	/***
	 *
	 * @param velocity: velocity of 4 beam
	 */
	void IntegrateDVLMeasurement2(const Eigen::Vector4d &velocity_beam, const double &dt);
	/*!
	 *
	 * @param v_di: V_di
	 */
	void SetVelocity(const cv::Point3d &v_di);
	void SetBeamOrientation(const Eigen::Vector4d &alpha, const Eigen::Vector4d &beta);
	void ReintegrateWithVelocity();
	void ReintegrateWithVelocity(const Eigen::Vector3d &velocity);
    void ReintegrateWithBias(const Bias &b);
	void ReintegrateWithBiasAndRotation(const Bias &b, const cv::Mat &R_g_d);
	void ReintegrateWithBiasRotationBeamOri(const Bias &b,
	                                        const cv::Mat &R_g_d,
	                                        const Eigen::Vector4d &alpha,
	                                        const Eigen::Vector4d &beta);
	void MergePrevious(DVLGroPreIntegration *pPrev);
	void MergePrevious2(DVLGroPreIntegration *pPrev);
	void SetNewBias(const Bias &bu_);
	IMU::Bias GetDeltaBias(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaRotation(const Bias &b_);
	cv::Mat GetDeltaRotation(const Bias &b, const cv::Mat &R_g_d);
	// equation(44)
	cv::Mat GetDeltaVelocity(const Bias &b_);
	// equation(44)
	cv::Mat GetDVLPosition(const Bias &b_);
    cv::Mat GetDVLPosition(const Bias &b_, const Eigen::Vector3d &velocity);
    cv::Mat GetDeltaPosition(const Bias &b_);
	cv::Mat GetDeltaPosition(const Bias &b, const cv::Mat &R_g_d);
	cv::Mat GetUpdatedDeltaRotation();
	cv::Mat GetUpdatedDeltaVelocity();
	cv::Mat GetUpdatedDeltaPosition();
	cv::Mat GetOriginalDeltaRotation();
	cv::Mat GetOriginalDeltaVelocity();
	cv::Mat GetOriginalDeltaPosition();
	Eigen::Matrix<double, 15, 15> GetInformationMatrix();
	cv::Mat GetDeltaBias();
	Bias GetOriginalBias();
	Bias GetUpdatedBias();
	void output();
	Eigen::Isometry3d getDVLPose();

public:
	double dT;
	cv::Mat C;
	cv::Mat Info;
	cv::Mat Nga, NgaWalk;

	// Values for the original bias (when integration was computed)
	Bias mb;
	Calib mCalib;
	/***
	 * dR: R_gi_gj. during integrtion R_gi_gk
	 * dV: V_di
	 * dP_dvl: t_di_didj, during integration t_di_didk
	 * dP_acc: t_bi_bibj, during integration t_bi_bibk
	 * dDeltaV: Delta_V_bi_bj = R_bi_w * (V_w_bj - V_w_bi - g_w * delta_t_i_j)
	 */
	cv::Mat dR, dV, dP_dvl, dDeltaV, dP_acc;
	// mVelocity: V_dk velocity reading in DVL frame at time k
	// mAngV: angular_v last angular v, this is used for DVL inegration
	cv::Mat mVelocity, mR_g_d;
	cv::Point3d mAngV;

	// DVL beam orientation
	Eigen::Vector4d mAlpha;
	Eigen::Vector4d mBeta;
	Eigen::Matrix<double, 4, 3> mE;
	Eigen::Matrix<double, 3, 3> mETEInv;

	/*** euqation(44)
	 * JRg: Jacobian of R wrt bias_gro
	 * JVg: Jacobian of v wrt bias_gro
	 * JVq: Jacobian of v wrt bias_acc
	 * JPg: Jacobian of p wrt bias_gro
	 * JPa: Jacobian of p wrt bias_acc
	 */
	cv::Mat JRg, JVg, JVa, JPg, JPa, JPv;
	cv::Mat avgA;
	cv::Mat avgW;

	// whether we have DVL reading
	bool bDVL;
	// if velocity reading is more then threshold, filter it out
	double mVelocityThreshold;
	cv::Point3d v_dk_dvl;

	void SetDVLDebugVelocity(const cv::Point3d &v_dk);
	Eigen::Vector3d v_di_dvl, v_dk_visual;
	std::vector<double> mBeams;


private:
	// Updated bias
	Bias bu;
	// Dif between original and updated bias
	// This is used to compute the updated values of the preintegration
	cv::Mat db;

	struct integrable
	{
        template<typename MatrixType, typename Archive>
        void serializeEigenMatrix(Archive &ar, MatrixType &m, const unsigned int version)
        {
            int cols = m.cols();
            int rows = m.rows();
            ar & cols;
            ar & rows;
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    ar & m(i, j);
                }
            }
        }

		template<class Archive>
		void serializePoint3d(Archive &ar, cv::Point3d &p, const unsigned int version)
		{
			double x, y, z;
			if (Archive::is_saving::value) {
				x = p.x;
				y = p.y;
				z = p.z;
				ar & x;
				ar & y;
				ar & z;
			}
			else if (Archive::is_loading::value) {
				ar & x;
				ar & y;
				ar & z;
				p = cv::Point3d(x, y, z);
			}
		}

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive &ar, const unsigned int version)
		{
			serializePoint3d(ar, a, version);
			serializePoint3d(ar, w, version);
			serializePoint3d(ar, v, version);
			serializeEigenMatrix(ar, v_beam, version);
			ar & t;
		}
		integrable()
		{}
		integrable(const cv::Point3d &a_, const cv::Point3d &w_, const double &t_)
			: a(a_), w(w_), t(t_)
		{}
		integrable(const cv::Point3d &a_,
		           const cv::Point3d &w_,
		           const cv::Point3d &v_,
		           const Eigen::Vector4d &v_beam,
		           const double &t_)
			: a(a_), w(w_), v(v_), v_beam(v_beam), t(t_)
		{}
		cv::Point3d a;
		cv::Point3d w;
		cv::Point3d v;
		Eigen::Vector4d v_beam;
		double t;
	};
public:
	std::vector<integrable> mvMeasurements;
	std::vector<integrable> mvMeasurements2;

	std::mutex mMutex;
};


#endif //DVLGROPREINTEGRATION_H
