#ifndef _CAR_CONTROLLER_H_
#define _CAR_CONTROLLER_H_

#include "CarPlannerCommon.h"
#include "LocalPlanner.h"


typedef std::list<ControlPlan*> PlanPtrList;

///////////////////////////////////////////////////////////////////////////////
class CarController
{
public:
    CarController();

    void Init(std::vector<MotionSample> &segmentSamples, LocalPlanner *pPlanner, BulletCarModel *pModel, double dt) ;
    void Reset();

    void GetCurrentCommands(const double time, ControlCommand &command);
    VehicleState GetCurrentPose();
    void SetCurrentPoseFromCarModel(BulletCarModel* pModel, int nWorldId);
    void SetCurrentPose(VehicleState pose, CommandList* pCommandList = NULL);
    void GetCurrentCommands(const double time,
                            ControlCommand& command,
                            Eigen::Vector3d& targetVel,
                            Sophus::SE3d &dT_target);
    double GetLastPlanStartTime();
    bool PlanControl(double dPlanStartTime, ControlPlan*& pPlanOut);
    static double AdjustStartingSample(const std::vector<MotionSample>& segmentSamples,
                                       VehicleState& state,
                                       int& segmentIndex,
                                       int& sampleIndex,
                                       int lowerLimit = 100,
                                       int upperLimit = 100);
    static void PrepareLookaheadTrajectory(const std::vector<MotionSample>& vSegmentSamples,
                                           ControlPlan *pPlan, VelocityProfile &trajectoryProfile, MotionSample &trajectorySample, const double dLookaheadTime);

private:
    bool _SampleControlPlan(ControlPlan* pPlan,LocalProblem& problem);
    bool _SolveControlPlan(const ControlPlan* pPlan, LocalProblem& problem, const MotionSample &trajectory);

    void _GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sampleIndex, double& interpolationAmount);



    int m_nLastCurrentPlanIndex;
    VehicleState m_CurrentState;
    BulletCarModel* m_pModel;
    LocalPlanner* m_pPlanner;
    ControlCommand m_LastCommand;

    double m_dStartTime;
    double m_dt;
    bool m_bStarted;
    bool m_bStopping;
    bool m_bPoseUpdated;
    bool m_bFirstPose;
    std::thread* m_pControlPlannerThread;

    PlanPtrList m_lControlPlans;
    std::vector<MotionSample> m_vSegmentSamples;
    CommandList m_lCurrentCommands;

    MotionSample m_MotionSample2dOnly;

    std::mutex m_PlanMutex;
    std::mutex m_PoseMutex;

    Eigen::Vector5d m_dLastDelta;

public:
    float m_dMaxControlPlanTime;
    float m_dLookaheadTime;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};


#endif

