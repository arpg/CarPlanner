#ifndef CARREGRESSOR_H
#define CARREGRESSOR_H

#include "CarPlannerCommon.h"
#include "LocalPlanner.h"
#include "threadpool.hpp"

#define NORM_NOT_INITIALIZED -1
#define REGRESSOR_NUM_THREADS 8
#define REGRESSOR_NUM_WORLDS 11

class CarRegressor
{
public:
    CarRegressor();
    void Init(double dEpsilon,std::fstream* pLogFile = NULL);
    void Regress(ApplyVelocitesFunctor5d& f, MotionSample &sample, const std::vector<RegressionParameter>& params, std::vector<RegressionParameter>& newParams);

    void ApplyParameters(ApplyVelocitesFunctor5d f,
                         MotionSample &plan,
                         const std::vector<RegressionParameter>& params,
                         const int index,
                         Eigen::Vector7f& errorOut,
                         int nStartIndex,
                         int nEndIndex,
                         std::vector<VehicleState> *pStatesOut = NULL,
                         CommandList *pPreviousCommands = NULL);

    void CalculateJacobian(ApplyVelocitesFunctor5d f, MotionSample &sample,
                            const std::vector<RegressionParameter>& params, std::vector<RegressionParameter>& vBestParams,
                            double& dBestNorm, int &nBestDimension, Eigen::MatrixXf &JtJ, Eigen::VectorXf &Jb);

    double CalculateParamNorms(ApplyVelocitesFunctor5d f,
                               MotionSample& plan,
                               const std::vector<RegressionParameter>& params,
                               std::vector<MotionSample> *pSamples = NULL,
                               std::vector<int> *pSampleIndices = NULL);

    Eigen::MatrixXf FiniteDiffFunctor(ApplyVelocitesFunctor5d f,
                                      MotionSample& plan,
                                      const std::vector<RegressionParameter>& params,
                                      Eigen::VectorXf& vBestParams,
                                      Eigen::Vector7f& dBaseError,
                                      double& dBestNorm,
                                      int& nBestDimension,
                                      int nStartIndex,
                                      int nEndIndex);



private:
    void _RefreshIndices(MotionSample &sample, ApplyVelocitesFunctor5d &f);
    void _OptimizeParametersGN(ApplyVelocitesFunctor5d& f, MotionSample& plan,
                               const std::vector<RegressionParameter>& dParams, std::vector<RegressionParameter>& dParamsOut,
                               double& dNewNorm);

    boost::threadpool::pool m_ThreadPool;
    double m_dEpsilon;
    double m_dCurrentNorm;
    OptimizationTask m_eCurrentTask;
    bool m_bFailed;
    int m_nSegmentLength;
    int m_nStartIndex;
    std::fstream* m_pLogFile;
    Eigen::MatrixXf m_dW;
    Eigen::MatrixXf m_dPrior;
    std::vector<std::pair<int,int> > m_vSegmentIndices;
};

#endif // CARREGRESSOR_H
