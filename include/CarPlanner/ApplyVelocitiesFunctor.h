#ifndef APPLYVELOCITIESFUNCTOR_H
#define APPLYVELOCITIESFUNCTOR_H

#include <vector>
#include <cmath>
#include <Eigen/LU>
#include <Eigen/Core>
#include <float.h>
#include "RpgUtils.h"
#include "CarPlannerCommon.h"
#include "BulletCarModel.h"
#include "CVarHelpers.h"
#include "cvars/CVar.h"
#include <boost/thread.hpp>
#include <queue>
#include "threadpool.hpp"

#define CAR_GRAVITY_COMPENSATION_COEFFICIENT 1.0
#define CAR_STEERING_COMPENSATION_COEFFICIENT 0

#define ROLL_TORQUE_CONTROL_ENABLED 0
#define PITCH_TORQUE_CONTROL_ENABLED 1
#define YAW_TORQUE_CONTROL_ENABLED 0


///////////////////////////////////////////////////////////////////////////////
struct ControlSample
{
public:
    double m_dSpeed;
    double m_dSteering;
    double m_Dt;
};

struct MotionSample
{
    std::vector<VehicleState> m_vStates;
    std::vector<ControlCommand> m_vCommands;

    double MaxCommandCurvature() const
    {
        double dMax = 0;//DBL_MIN;
        for(size_t ii = 0 ; ii < m_vCommands.size() ; ii++){
            //dMax = std::max(dMax,m_vCommands[ii].m_dCurvature);
            dMax += m_vCommands[ii].m_dCurvature;
        }
        return dMax;
    }

    CommandList GetDelayedCommandList(const double& delay, const int& nStartIndex)
    {
        CommandList prevCommands;
        double totalDelay = delay;
        for(int kk = nStartIndex ; kk >= 0 && totalDelay > 0 ; kk--) {
            prevCommands.push_back(m_vCommands[kk]);
            totalDelay -= m_vCommands[kk].m_dT;
        }
        return prevCommands;
    }

    const VehicleState& GetLastPose() const { return m_vStates.back(); }

    std::vector<Sophus::SE3f> GetMotionSample() const
    {
        std::vector<Sophus::SE3f> vPoses;
        vPoses.reserve(m_vStates.size());
        for(const VehicleState& state : m_vStates){
            vPoses.push_back(state.m_dTwv);
        }
        return vPoses;
    }

    double GetBadnessCost() const
    {
        double cost = 0;
        if(m_vCommands.size() != 0){
            const ControlCommand* pPrevCommand = &m_vCommands.front();
            for(size_t ii = 1; ii < m_vStates.size() ; ii++){
                //const VehicleState& state = m_vStates[ii];
                const ControlCommand& command = m_vCommands[ii];
                //cost = std::max(state.m_dV.norm() * state.m_dW.norm(),cost);
                //cost += fabs(state.m_dV.norm() * state.m_dW[2]) - fabs(m_vCommands[ii].m_dCurvature);
                cost += fabs(command.m_dPhi - pPrevCommand->m_dPhi);
                pPrevCommand = &m_vCommands[ii];
                //cost += fabs(state.m_dSteering);
            }
            cost /= GetDistance();
        }
        return cost;
    }

    double GetDistance() const
    {
        double dist = 0;
        if(m_vStates.empty() == false){
            Eigen::Vector3f lastPos = m_vStates[0].m_dTwv.translation();
            for(const VehicleState& state : m_vStates){
                dist += (state.m_dTwv.translation()-lastPos).norm();
                lastPos = state.m_dTwv.translation();
            }
        }
        return dist;
    }

    ////////////////////////////////////////////////////////////////
    static bool FixSampleIndexOverflow(const std::vector<MotionSample>& segmentSamples, int& segmentIndex, int& sampleIndex, bool loop = true)
    {
        bool overFlow = false;
        bool underFlow = false;

        //if this index is beyond the bounds, we move to the next segment
        while(sampleIndex >= (int)segmentSamples[segmentIndex].m_vStates.size()) {
            sampleIndex -= (int)segmentSamples[segmentIndex].m_vStates.size();
            segmentIndex++;

            //if we have reached the end of the segments, we can loop back
            if(segmentIndex >= (int)segmentSamples.size()) {
                if(loop){
                    //loop around
                    segmentIndex = 0;
                }else{
                    //do not loop around
                    segmentIndex = segmentSamples.size()-1;
                    sampleIndex = (int)segmentSamples[segmentIndex].m_vStates.size();
                }
            }
            overFlow = true;
        }

        while(sampleIndex < 0) {
            segmentIndex--;

            //if we have reached the beginning of the segments, we can loop back
            if(segmentIndex < 0) {
                segmentIndex = segmentSamples.size()-1;
            }

            sampleIndex += (int)segmentSamples[segmentIndex].m_vStates.size();
            underFlow = true;
        }

        return overFlow || underFlow;
    }

    void Clear(){
        m_vStates.clear();
        m_vCommands.clear();
    }
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ControlPlan
{
public:

    double m_dStartTime;
    double m_dEndTime;
    double m_dNorm;
    MotionSample m_Sample;

    Sophus::SE3f m_dStartPose;
    Sophus::SE3f m_dEndPose;

    int m_nStartSegmentIndex;   //the segment at which this plan starts
    int m_nStartSampleIndex; //the sample in the segment at which this control plan starts

    int m_nEndSegmentIndex;   //the segment at which this plan ends
    int m_nEndSampleIndex; //the sample in the segment at which this control plan ends
    int m_nPlanId;


    VehicleState m_StartState;
    Eigen::Vector3d m_dStartTorques;
    VehicleState m_GoalState;

    void Clear() {
        m_Sample.Clear();
    }

    ~ControlPlan(){
        //dout("Deleting control plan.");
    }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ApplyVelocitesFunctor5d
{
public:
    ApplyVelocitesFunctor5d(BulletCarModel *pCarModel,
                            Eigen::Vector3f dInitTorques,
                            CommandList* pPreviousCommands = NULL);

    VehicleState ApplyVelocities(const VehicleState &startState,
                                 MotionSample& sample,
                                 int nIndex = 0,
                                 bool noCompensation = false);

    void ApplyVelocities(const VehicleState &startingState,
                         std::vector<ControlCommand> &m_vCommands,
                         std::vector<VehicleState>& vStatesOut,
                         const int nStartIndex,
                         const int nEndIndex,
                         const int nIndex,
                         const bool noCompensation = false,
                         const CommandList *pPreviousCommands = NULL);


    double GetMaxWheelTorque(const VehicleState& state, const int nIndex);
    double GetGravityCompensation(int nIndex);
    double GetSteeringCompensation(VehicleState& state, double phi, double curvature, int nIndex);
    double GetFrictionCompensation(int nIndex, double dt);

    BulletCarModel* GetCarModel(){ return m_pCarModel; }
    const BulletCarModel* GetCarModel() const{ return m_pCarModel; }
    CommandList& GetPreviousCommand() { return m_lPreviousCommands; }
    void SetPreviousCommands(const CommandList& list) { m_lPreviousCommands = list;}
    void ResetPreviousCommands() { return m_lPreviousCommands.clear(); }
    bool SetNoDelay(bool bNoDelay){ return (m_bNoDelay = bNoDelay); }
private:
    BulletCarModel *m_pCarModel;
    Eigen::Vector3f m_dInitTorques;
    CommandList m_lPreviousCommands;
    bool m_bNoDelay;
};
#endif // APPLYVELOCITIESFUNCTOR_H
