/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef IMUTYPES_H
#define IMUTYPES_H

#include<vector>
#include<utility>
#include<opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3
{

namespace IMU
{

const float GRAVITY_VALUE = 9.81;

//IMU measurement (gyro, accelerometer and timestamp)
class ImuPoint
{
public:
	ImuPoint(const float &acc_x, const float &acc_y, const float &acc_z,
	         const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
	         const double &timestamp)
		: a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z), t(timestamp)
	{}
	ImuPoint(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp)
		:
		a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), t(timestamp)
	{}
public:
	cv::Point3f a;
	cv::Point3f w;
	double t;
};

class DvlPoint
{
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
	void serializeEigenV4d(Archive &ar, Eigen::Vector4d &v, const unsigned int version)
	{
		double x, y, z, w;
		if (Archive::is_saving::value) {
			x = v.x();
			y = v.y();
			z = v.z();
			w = v.w();
			ar & x;
			ar & y;
			ar & z;
			ar & w;
		}
		else if (Archive::is_loading::value) {
			ar & x;
			ar & y;
			ar & z;
			ar & w;
			v = Eigen::Vector4d(x, y, z, w);
		}
	}

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		serializePoint3f(ar,v,version);
		serializeEigenV4d(ar,vb,version);
		ar & t;
	}
public:
	DvlPoint(){}
	DvlPoint(const float &v_x, const float &v_y, const float &v_z, const double &timestamp)
		: v(v_x, v_y, v_z), t(timestamp)
	{}
	DvlPoint(const cv::Point3f vel, const double &timestamp)
		:
		v(vel.x, vel.y, vel.z), t(timestamp)
	{}
	DvlPoint(const float &v_x,
	         const float &v_y,
	         const float &v_z,
	         const float &vb_0,
	         const float &vb_1,
	         const float &vb_2,
	         const float &vb_3,
	         const double &timestamp)
		: v(v_x, v_y, v_z), vb(vb_0, vb_1, vb_2, vb_3), t(timestamp)
	{}
public:
	cv::Point3f v;
	Eigen::Vector4d vb;
	double t;
};

class GyroDvlPoint
{
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
	void serializeEigenV4d(Archive &ar, Eigen::Vector4d &v, const unsigned int version)
	{
		double x, y, z, w;
		if (Archive::is_saving::value) {
			x = v.x();
			y = v.y();
			z = v.z();
			w = v.w();
			ar & x;
			ar & y;
			ar & z;
			ar & w;
		}
		else if (Archive::is_loading::value) {
			ar & x;
			ar & y;
			ar & z;
			ar & w;
			v = Eigen::Vector4d(x, y, z, w);
		}
	}

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		serializePoint3f(ar, angular_v, version);
		serializePoint3f(ar, acc, version);
		serializePoint3f(ar, v, version);
		serializeEigenV4d(ar, vb, version);
		ar & t;
	}
public:
	GyroDvlPoint()
	{}
	GyroDvlPoint(const double &a_x,
	             const double &a_y,
	             const double &a_z,
	             const double &v_x,
	             const double &v_y,
	             const double &v_z,
	             const double &vb_0,
	             const double &vb_1,
	             const double &vb_2,
	             const double &vb_3,
	             const double &timestamp)
		: angular_v(a_x, a_y, a_z), v(v_x, v_y, v_z), vb(vb_0, vb_1, vb_2, vb_3), t(timestamp)
	{}
	GyroDvlPoint(const double &acc_x,
	             const double &acc_y,
	             const double &acc_z,
	             const double &av_x,
	             const double &av_y,
	             const double &av_z,
	             const double &v_x,
	             const double &v_y,
	             const double &v_z,
	             const double &vb_0,
	             const double &vb_1,
	             const double &vb_2,
	             const double &vb_3,
	             const double &timestamp)
		: angular_v(av_x, av_y, av_z), acc(acc_x, acc_y, acc_z), v(v_x, v_y, v_z), vb(vb_0, vb_1, vb_2, vb_3),
		  t(timestamp)
	{}
public:
	cv::Point3d angular_v;
	cv::Point3d acc;
	cv::Point3d v;
	Eigen::Vector4d vb;
	double t;
};

//IMU biases (gyro and accelerometer)
class Bias
{
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & bax;
		ar & bay;
		ar & baz;

		ar & bwx;
		ar & bwy;
		ar & bwz;
	}

public:
	Bias()
		: bax(0), bay(0), baz(0), bwx(0), bwy(0), bwz(0)
	{}
	Bias(const double &b_acc_x, const double &b_acc_y, const double &b_acc_z,
	     const double &b_ang_vel_x, const double &b_ang_vel_y, const double &b_ang_vel_z)
		:
		bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z)
	{}
	void CopyFrom(Bias &b);
	friend std::ostream &operator<<(std::ostream &out, const Bias &b);

public:
	double bax, bay, baz;
	double bwx, bwy, bwz;
};

//IMU calibration (Tbc, Tcb, noise)
class Calib
{
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
	void serialize(Archive &ar, const unsigned int version)
	{
		// serializeMatrix(ar, Tcb, version);
		// serializeMatrix(ar, Tbc, version);
		// serializeMatrix(ar, Cov, version);
		// serializeMatrix(ar, CovWalk, version);
		serializeMatrix(ar, mT_gyro_c, version);
		serializeMatrix(ar, mT_c_gyro, version);
		serializeMatrix(ar, mT_dvl_c, version);
		serializeMatrix(ar, mT_c_dvl, version);
		serializeMatrix(ar, mT_gyro_dvl, version);
	}

public:
	Calib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
	{
		Set(Tbc_, ng, na, ngw, naw);
	}
	Calib(cv::Mat T_gyro_c, cv::Mat T_dvl_c)
		: mT_gyro_c(T_gyro_c), mT_dvl_c(T_dvl_c)
	{
		mT_c_gyro = cv::Mat::eye(4, 4, CV_32F);
		mT_c_gyro.rowRange(0, 3).colRange(0, 3) = mT_gyro_c.rowRange(0, 3).colRange(0, 3).t();
		mT_c_gyro.rowRange(0, 3).col(3) =
			-mT_gyro_c.rowRange(0, 3).colRange(0, 3).t() * mT_gyro_c.rowRange(0, 3).col(3);

		mT_c_dvl = cv::Mat::eye(4, 4, CV_32F);
		mT_c_dvl.rowRange(0, 3).colRange(0, 3) = mT_dvl_c.rowRange(0, 3).colRange(0, 3).t();
		mT_c_dvl.rowRange(0, 3).col(3) = -mT_dvl_c.rowRange(0, 3).colRange(0, 3).t() * mT_dvl_c.rowRange(0, 3).col(3);

		mT_gyro_dvl = mT_gyro_c * mT_c_dvl;
	}
    Calib(cv::Mat T_gyro_c, cv::Mat T_dvl_c, const float &ng, const float &na, const float &ngw, const float &naw)
            : mT_gyro_c(T_gyro_c.clone()), mT_dvl_c(T_dvl_c.clone())
    {
        mT_c_gyro = cv::Mat::eye(4, 4, CV_32F);
        mT_c_gyro.rowRange(0, 3).colRange(0, 3) = mT_gyro_c.rowRange(0, 3).colRange(0, 3).t();
        mT_c_gyro.rowRange(0, 3).col(3) =
                -mT_gyro_c.rowRange(0, 3).colRange(0, 3).t() * mT_gyro_c.rowRange(0, 3).col(3);

        mT_c_dvl = cv::Mat::eye(4, 4, CV_32F);
        mT_c_dvl.rowRange(0, 3).colRange(0, 3) = mT_dvl_c.rowRange(0, 3).colRange(0, 3).t();
        mT_c_dvl.rowRange(0, 3).col(3) = -mT_dvl_c.rowRange(0, 3).colRange(0, 3).t() * mT_dvl_c.rowRange(0, 3).col(3);

        mT_gyro_dvl = mT_gyro_c * mT_c_dvl;

        Set(T_gyro_c, ng, na, ngw, naw);
    }
	Calib(const Calib &calib)
		:
		mT_gyro_c(calib.mT_gyro_c.clone()), mT_dvl_c(calib.mT_dvl_c.clone()), mT_c_gyro(calib.mT_c_gyro.clone()),
		mT_c_dvl(calib.mT_c_dvl.clone()),
		mT_gyro_dvl(calib.mT_gyro_dvl.clone())
	{
		Tbc = calib.Tbc.clone();
		Tcb = calib.Tcb.clone();
		Cov = calib.Cov.clone();
		CovWalk = calib.CovWalk.clone();

	}
	Calib()
	{
	}

	void Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);
    void SetExtrinsic(const cv::Mat& T_gyro_c, const cv::Mat& T_dvl_c);

public:
	cv::Mat Tcb;
	cv::Mat Tbc;
	cv::Mat Cov, CovWalk;
	// transformation from gyroscope to camera
	// set translation as 0 0 0
	// only rotation
	cv::Mat mT_gyro_c;
	cv::Mat mT_c_gyro;
	// transformation from dvl to camera
	cv::Mat mT_dvl_c;
	cv::Mat mT_c_dvl;
	cv::Mat mT_gyro_dvl;
};

//Integration of 1 gyro measurement
class IntegratedRotation
{
public:
	IntegratedRotation()
	{}
	IntegratedRotation(const cv::Point3d &angVel, const Bias &imuBias, const double &time);

public:
	float deltaT; //integration time
	cv::Mat deltaR; //integrated rotation
	cv::Mat rightJ; // right jacobian
};

//Preintegration of Imu Measurements
class Preintegrated
{
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
	void serialize(Archive &ar, const unsigned int version)
	{
		ar & dT;
		serializeMatrix(ar, C, version);
		serializeMatrix(ar, Info, version);
		serializeMatrix(ar, Nga, version);
		serializeMatrix(ar, NgaWalk, version);
		ar & b;
		serializeMatrix(ar, dR, version);
		serializeMatrix(ar, dV, version);
		serializeMatrix(ar, dP, version);
		serializeMatrix(ar, JRg, version);
		serializeMatrix(ar, JVg, version);
		serializeMatrix(ar, JVa, version);
		serializeMatrix(ar, JPg, version);
		serializeMatrix(ar, JPa, version);
		serializeMatrix(ar, avgA, version);
		serializeMatrix(ar, avgW, version);

		ar & bu;
		serializeMatrix(ar, db, version);
		ar & mvMeasurements;
	}

public:
	Preintegrated(const Bias &b_, const Calib &calib);
	Preintegrated(Preintegrated *pImuPre);
	Preintegrated()
	{}
	~Preintegrated()
	{}
	void CopyFrom(Preintegrated *pImuPre);
	void Initialize(const Bias &b_);
	void IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt);
	void Reintegrate();
	void MergePrevious(Preintegrated *pPrev);
	void SetNewBias(const Bias &bu_);
	IMU::Bias GetDeltaBias(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaRotation(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaVelocity(const Bias &b_);
	// equation(44)
	cv::Mat GetDeltaPosition(const Bias &b_);
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

public:
	float dT;
	cv::Mat C;
	cv::Mat Info;
	cv::Mat Nga, NgaWalk;

	// Values for the original bias (when integration was computed)
	Bias b;
	/***
	 * dR: delta ~R_i_j in equation(35)
	 * dV: delta ~v_i_j in equation(36)
	 * dP: delta ~p_i_j in equation(37)
	 */
	cv::Mat dR, dV, dP;
	/*** euqation(44)
	 * JRg: Jacobian of R wrt bias_gro
	 * JVg: Jacobian of v wrt bias_gro
	 * JVq: Jacobian of v wrt bias_acc
	 * JPg: Jacobian of p wrt bias_gro
	 * JPa: Jacobian of p wrt bias_acc
	 */
	cv::Mat JRg, JVg, JVa, JPg, JPa;
	cv::Mat avgA;
	cv::Mat avgW;


private:
	// Updated bias
	Bias bu;
	// Dif between original and updated bias
	// This is used to compute the updated values of the preintegration
	cv::Mat db;

	struct integrable
	{
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

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive &ar, const unsigned int version)
		{
			serializePoint3f(ar, a, version);
			serializePoint3f(ar, w, version);
			ar & t;
		}

		integrable()
		{}
		integrable(const cv::Point3f &a_, const cv::Point3f &w_, const float &t_)
			: a(a_), w(w_), t(t_)
		{}
		cv::Point3f a;
		cv::Point3f w;
		float t;
	};

	std::vector<integrable> mvMeasurements;

	std::mutex mMutex;
};

// Lie Algebra Functions
cv::Mat ExpSO3(const float &x, const float &y, const float &z);
Eigen::Matrix<double, 3, 3> ExpSO3(const double &x, const double &y, const double &z);
cv::Mat ExpSO3(const cv::Mat &v);
cv::Mat LogSO3(const cv::Mat &R);
cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat RightJacobianSO3(const cv::Mat &v);
cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat InverseRightJacobianSO3(const cv::Mat &v);
cv::Mat Skew(const cv::Mat &v);
cv::Mat NormalizeRotation(const cv::Mat &R);

}

} //namespace ORB_SLAM2

#endif // IMUTYPES_H
