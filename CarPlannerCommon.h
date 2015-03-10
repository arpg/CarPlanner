#ifndef CARPLANNERCOMMON_H
#define CARPLANNERCOMMON_H
#define DEBUG 1

#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include<Eigen/StdVector>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <sys/time.h>
#include <time.h>
#include "boost/timer/timer.hpp"
#include <boost/format.hpp>
#include "sophus/se3.hpp"

#ifdef __APPLE__
#include "pthread.h"
#include <mach/clock.h>
#include <mach/mach.h>
#define SetThreadName(x) pthread_setname_np(x);
#else
#include <sys/prctl.h>
#define SetThreadName(x) prctl(PR_SET_NAME,x,0,0,0);
#endif

#define DEBUG 1
#ifdef DEBUG
#define dout(str) std::cout << __FUNCTION__ << " --  " << str << std::endl
#define dout_cond(str,x) if(x)std::cout << __FUNCTION__ << " --  " << str << std::endl
#else
#define dout(str)
#endif

//#define CAR_HEIGHT_OFFSET 0.06
#define VICON_CAR_HEIGHT_OFFSET 0.02

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


enum MochaCommands{
    eMochaPause = 1,
    eMochaSolve = 2,
    eMochaRestart = 3,
    eMochaStep = 4,
    eMochaLearn = 5,
    eMochaToggleTrajectory = 6,
    eMochaTogglePlans = 7,
    eMochaLoad = 8,
    eMochaClear = 9,
    eMochaFixGround = 10,
    eMochaPpmControl = 11,
    eMochaSimulationControl = 12,

    eMochaLeft = 90,
    eMochaRight = 91,
    eMochaUp = 92,
    eMochaDown = 93
};

enum OptimizationTask {
    eGaussNewton = 0,
    eDiscrete = 1,
    eDiscreteSearch = 2
};

enum PpmChannels
{
    ePpm_Accel = 3,
    ePpm_Steering = 2
};

enum WaypointType
{
    eWaypoint_Normal = 1,
    eWayoiint_FlipFront = 2
};

enum eLocType { VT_AIR, VT_GROUND, VT_REC_RAMP, VT_CIR_RAMP };

// add Vector5d to eigen namespace
#define CURVE_DIM 4 //number of dimensions for the curve
#define ACCEL_INDEX 4 //the index of the acceleration
#define POSE2D_DIM 4
#define POSE_DIM 5


#define SERVO_RANGE 500.0

namespace Eigen {
    typedef Matrix<double,6,1> VectorPose3D;
    typedef Matrix<double,POSE_DIM,1> VectorPose; // defined as X,Y,Theta,Curvature,V
    typedef Matrix<double,POSE_DIM-1,1> VectorError; // defined as X,Y,Theta,V
    typedef Matrix<double,CURVE_DIM,1> VectorCubic2D;
    typedef Matrix<double,POSE2D_DIM,1> VectorPose2D;
    typedef Matrix<double, POSE_DIM-1,POSE_DIM-1> MatrixJtJ;
    typedef Matrix<double, POSE_DIM-1,POSE_DIM-1> MatrixWeights;
    typedef Matrix<double, 3,4> MatrixCubicJac;
    typedef Matrix<double, 4,4> MatrixCubicJac_k;


    typedef Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRowMaj;
    typedef Matrix<double,5,1> Vector5d ;
    typedef Matrix<double,6,1> Vector6d ;
    typedef Matrix<double,7,1> Vector7d ;

    typedef Matrix<double,6,6> Matrix6d;


    typedef std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Vector6dAlignedVec;
    typedef std::vector<Eigen::Vector5d,Eigen::aligned_allocator<Eigen::Vector5d> > Vector5dAlignedVec;
    typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dAlignedVec;
    typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dAlignedVec;
    typedef std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > Vector4dAlignedVec;
    typedef std::vector<Eigen::Vector7d,Eigen::aligned_allocator<Eigen::Vector7d> > Vector7dAlignedVec;
    typedef std::vector<Eigen::VectorXd,Eigen::aligned_allocator<Eigen::VectorXd> > VectorXdAlignedVec;
    typedef std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > Matrix4dAlignedVec;
    typedef std::vector<Eigen::MatrixXdRowMaj,Eigen::aligned_allocator<Eigen::MatrixXdRowMaj> > MatrixXdRowMajAlignedVec;

    typedef std::vector<std::pair<Eigen::VectorPose,Eigen::VectorPose> > ErrorVec;
}

template<typename T>
inline const Eigen::Matrix<T,3,1> GetBasisVector(Sophus::SE3Group<T> rot, int n){ return rot.matrix().block<3,1>(0,n); }
inline const Eigen::Vector3d GetBasisVector(Sophus::SE3d rot, int n){ return rot.matrix().block<3,1>(0,n); }
template<typename T>
inline const Eigen::Matrix<T,3,1> GetBasisVector(Sophus::SO3Group<T> rot, int n){ return rot.matrix().block<3,1>(0,n); }
inline const Eigen::Vector3d GetBasisVector(Sophus::SO3d rot, int n){ return rot.matrix().block<3,1>(0,n); }


inline void current_utc_time(struct timespec *ts) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
#else
  clock_gettime(CLOCK_REALTIME, ts);
#endif

}




/////////////////////////////////////////////////////////////////////////////////////////
inline double SoftMaximum(double x, double y,const double multiplier = 1.0)
{
    x *= multiplier;
    y *= multiplier;
    double maximum = std::max(x, y);
    double minimum = std::min(x, y);
    return (log1p(exp(minimum-maximum)) + maximum)/multiplier;
}

/////////////////////////////////////////////////////////////////////////////////////////
inline double SoftMinimum(double x, double y, const double multiplier = 1.0)
{
    return -SoftMaximum(-x,-y,multiplier);
}



//static boost::timer::cpu_timer g_cpuTimer;

// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Tic() {

    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec  + 1e-6 * (tv.tv_usec);

    //struct timespec tv;
    //current_utc_time(&tv);
    //return tv.tv_sec + 1e-9 * (tv.tv_nsec);

    //boost::timer::cpu_times const elapsed_times(g_cpuTimer.elapsed());
    //boost::timer::nanosecond_type const elapsed(elapsed_times.system+ elapsed_times.user);
    //return elapsed*1e9;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double RealTime() {
    return Tic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Toc(double dTic) {
    return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double TocMS(double dTic) {
    return ( Tic() - dTic)*1000.;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
inline T powi(T num, int exp) {
    if(exp == 0 ){
        return 1;
    }else if( exp < 0 ) {
        return 0;
    }else{
        T ret = num;
        for(int ii = 1; ii < exp ; ii++) {
            ret *= num;
        }
        return ret;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double powi(double num, int exp) { return powi<double>(num,exp); }



////////////////////////////////////////////////////////////////////////////

template <class Derived> std::fstream& operator>>(std::fstream& fs, Eigen::MatrixBase<Derived>& mM) {
    typedef typename Eigen::internal::traits<Derived>::Scalar LScalar;
    std::string sLine;
    int nHeight = 0;
    int nWidth;
    std::vector<LScalar> vVals;
    LScalar dVal;
    while (getline(fs, sLine)) {
        std::stringstream ss(std::stringstream::in |
                std::stringstream::out);
        ss << sLine;
        int count = 0;
        while (ss >> dVal) {
            count++;
            vVals.push_back(dVal);
        }

        if (nHeight == 0) nWidth = vVals.size();
        nHeight++;
    }
    if (int(vVals.size()) != nWidth * nHeight) {
        printf("ERROR: while reading matrix from file, missing data.\n");
        return fs;
    }
    mM = Derived(nHeight, nWidth);
    for (int nRow = 0; nRow < nHeight; nRow++) {
        for (int nCol = 0; nCol < nWidth; nCol++) {
            mM(nRow, nCol) = vVals[ nRow * nWidth + nCol ];
        }
    }
    return fs;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Tinv(const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d res = T;
    res.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
    res.block<3,1>(0,3) = -(T.block<3,3>(0,0).transpose())*T.block<3,1>(0,3);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Tinv_2D(const Eigen::Matrix3d& T)
{
    Eigen::Matrix3d res = T;
    res.block<2,2>(0,0) = T.block<2,2>(0,0).transpose();
    res.block<2,1>(0,2) = -(T.block<2,2>(0,0).transpose())*T.block<2,1>(0,2);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Cart2T_2D(const Eigen::Vector3d& cart)
{
    Eigen::Matrix3d res;
    res << cos(cart[2]), -sin(cart[2]), cart[0],
           sin(cart[2]),  cos(cart[2]), cart[1],
           0           ,  0           ,       1;
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d T2Cart_2D(const Eigen::Matrix3d& T)
{
    Eigen::Vector3d res;
    res[0] = T(0,2);
    res[1] = T(1,2);
    res[2] = atan2(T(1,0),T(0,0));
    return res;
}


#endif // CARPLANNERCOMMON_H
