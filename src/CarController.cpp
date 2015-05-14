#include "CarPlanner/CarController.h"


static bool& g_bShow2DResult = CVarUtils::CreateGetUnsavedCVar("debug.Show2DResult",false);
static bool& g_bOptimize2DOnly = CVarUtils::CreateGetUnsavedCVar("debug.Optimize2DOnly",false);
static bool& g_bForceZeroStartingCurvature = CVarUtils::CreateGetUnsavedCVar("debug.ForceZeroStartingCurvature",false);
static double& g_dMinLookaheadTime(CVarUtils::CreateGetUnsavedCVar("debug.MinLookaheadTime",(double)0.05,""));
static double& g_dMaxLookaheadTime(CVarUtils::CreateGetUnsavedCVar("debug.MaxLookaheadTime",(double)2.0,""));
static double& g_dInitialLookaheadTime(CVarUtils::CreateGetUnsavedCVar("debug.InitialLookaheadTime",(double)0.5,""));
static double& g_dMaxPlanTimeLimit(CVarUtils::CreateGetUnsavedCVar("debug.MaxPlanTimeLimit",(double)1.0,""));
static double& g_dLookaheadEmaWeight(CVarUtils::CreateGetUnsavedCVar("debug.LookaheadEmaWeight",1.0,""));
static bool& g_bFreezeControl(CVarUtils::CreateGetUnsavedCVar("debug.FreezeControl",false,""));
static bool& g_bPointCost(CVarUtils::CreateGetUnsavedCVar("debug.PointCost",false,""));
static bool& g_bInertialControl = CVarUtils::CreateGetUnsavedCVar("debug.InertialControl",false);
static bool& g_bInfiniteTime = CVarUtils::CreateGetUnsavedCVar("debug.InfiniteTime",false);
static bool& g_bFrontFlip = CVarUtils::CreateGetUnsavedCVar("debug.FrontFlip",false);
static double& g_dMaxPlanNorm = CVarUtils::CreateGetUnsavedCVar("debug.MaxPlanNorm",5.0);


/////////////////////////////////////////////////////////////////////////////////////////
CarController::CarController() :
    m_dMaxControlPlanTime(CVarUtils::CreateGetCVar("controller.MaxControlPlanTime",(float)0.2,"")),
    m_dLookaheadTime(CVarUtils::CreateUnsavedCVar("controller.LookaheadTime",(float)0.2,"")),
    m_pControlPlannerThread(NULL)
{
    m_dLastDelta.setZero();
    //m_vControlPlans.reserve(10);
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::Init(std::vector<MotionSample>& segmentSamples,LocalPlanner *pPlanner, BulletCarModel *pModel, double dt) {
    m_vSegmentSamples = segmentSamples;
    m_pModel = pModel;
    m_pPlanner = pPlanner;
    m_bStopping = false;
    m_bStarted = false;
    m_bFirstPose = true;
    //m_pCurrentPlan = NULL;
    m_dt = dt;
    m_bPoseUpdated = false;
}


/////////////////////////////////////////////////////////////////////////////////////////
void CarController::Reset()
{
    {
        std::unique_lock<std::mutex> lock(m_PlanMutex);
        while(m_lControlPlans.begin() != m_lControlPlans.end()) {
            //delete this plan
            delete(m_lControlPlans.front());
            m_lControlPlans.erase(m_lControlPlans.begin());
        }
    }

    m_LastCommand = ControlCommand();
    m_bFirstPose = true;
    m_bStopping = false;
    m_bStarted = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::_SampleControlPlan(ControlPlan* pPlan,LocalProblem& problem)
{
    //get the motion sample for the new control plan
    if(g_bOptimize2DOnly == true){
        pPlan->m_Sample.m_vCommands = m_MotionSample2dOnly.m_vCommands;
    }else{
        pPlan->m_Sample.m_vCommands = problem.m_pBestSolution->m_Sample.m_vCommands;
    }

    if(g_bShow2DResult) {
        //ONLY FOR VISUALIZATION. REMOVE WHEN NO LONGER NEEDED
       Eigen::Vector3dAlignedVec samples;

        m_pPlanner->SamplePath(problem,samples,true);
        pPlan->m_Sample.m_vStates.reserve(samples.size());
        for(const Eigen::Vector3d& pos : samples){
            Sophus::SE3d Twv(Sophus::SO3d(),pos);
            pPlan->m_Sample.m_vStates.push_back(VehicleState(Twv,0));
        }
    }else{
        problem.m_pFunctor->ApplyVelocities(pPlan->m_StartState,
                                            pPlan->m_Sample.m_vCommands,
                                            pPlan->m_Sample.m_vStates,
                                            0,
                                            pPlan->m_Sample.m_vCommands.size(),
                                            0,
                                            true);
        //if we are in the air, make sure no force is applied and the wheels are straight
        for(size_t ii = 0 ; ii < pPlan->m_Sample.m_vStates.size() ; ii++){
            if(pPlan->m_Sample.m_vStates[ii].IsAirborne()){
                if(g_bFrontFlip){
                    pPlan->m_Sample.m_vCommands[ii].m_dForce = 0;
                }else{
                    pPlan->m_Sample.m_vCommands[ii].m_dForce = 0 + problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::AccelOffset]*SERVO_RANGE;
                }
                pPlan->m_Sample.m_vCommands[ii].m_dPhi = 0 + problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::SteeringOffset]*SERVO_RANGE;
            }
        }
    }


    //get the plan times in order by offsetting them by the start time
    for(VehicleState& state: pPlan->m_Sample.m_vStates) {
        state.m_dTime += pPlan->m_dStartTime;
    }

    for(ControlCommand& command: pPlan->m_Sample.m_vCommands) {
        command.m_dTime += pPlan->m_dStartTime;
    }

    if(pPlan->m_Sample.m_vCommands.empty()) {
        dout("Empty control plan discovered...");
        return false;
    }

    pPlan->m_dEndTime = pPlan->m_Sample.m_vStates.back().m_dTime;
    //set the norm on the plan
    pPlan->m_dNorm = problem.m_CurrentSolution.m_dNorm;

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::_SolveControlPlan(const ControlPlan* pPlan,LocalProblem& problem,const MotionSample& trajectory)
{
    bool res = m_pPlanner->InitializeLocalProblem(problem,pPlan->m_dStartTime,&problem.m_vVelProfile,g_bPointCost ? eCostPoint : eCostTrajectory);
    problem.m_bInertialControlActive = g_bInertialControl;
    problem.m_Trajectory = trajectory;

    if( res == false ){
        dout("2d planner failed to converge...");
        return false;
    }

    res = true;

    std::chrono::high_resolution_clock::time_point timer = std::chrono::high_resolution_clock::now();

    while(1)
    {
        //make sure the plan is not fully airborne
        //bool isAirborne = (pPlan->m_StartState.IsAirborne() && pPlan->m_GoalState.IsAirborne());
        if(g_bOptimize2DOnly /*|| isAirborne*/) {
            m_pPlanner->SimulateTrajectory(m_MotionSample2dOnly,problem,0,true);
            break;
        }else{
            if( (m_pPlanner->Iterate(problem)) == true ) {
                break;
            }
        }

        if(m_bStopping){
            res = false;
        }

        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timer - now).count();
        if(g_bInfiniteTime){
            if( elapsed > 1e6*g_dMaxPlanTimeLimit){
                break;
            }
        }else{
            if( elapsed > 1e6*(m_dMaxControlPlanTime*m_dLookaheadTime)){
                break;
            }
        }
    }

    //and now obtain the new delta
    m_dLastDelta = problem.m_CurrentSolution.m_dOptParams - problem.m_dInitOptParams;

    if(problem.m_CurrentSolution.m_dNorm > g_dMaxPlanNorm){
        dout("Planned control plan with norm too high -> " << problem.m_CurrentSolution.m_dNorm  );
        res = false;
    }

    return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::PlanControl(double dPlanStartTime, ControlPlan*& pPlanOut) {
    try
    {
        pPlanOut = NULL;
        int nCurrentSampleIndex;
        double interpolationAmount;
        double planStartCurvature;
        Eigen::Vector3d planStartTorques = Eigen::Vector3d::Zero();
        PlanPtrList::iterator nCurrentPlanIndex;
        ControlPlan* pPlan = NULL;

        //reset the starting position
        int oldPlanStartSegment = 0;
        int oldPlanStartSample = 0;
        int sampleCount = 0;

        //only continue planning if the pose has been updated since the last plan
        {
            std::unique_lock<std::mutex> lock(m_PoseMutex);
            if(m_bPoseUpdated == false) {
                //dout("Pose not updated, exiting control.");
                return false;
            }else{
                m_bPoseUpdated = false;
            }
        }


        pPlan = new ControlPlan();

        {
            std::unique_lock<std::mutex> lock(m_PlanMutex);

            //first find out where we are on the current plan
            _GetCurrentPlanIndex(dPlanStartTime,nCurrentPlanIndex,nCurrentSampleIndex,interpolationAmount);

            if(nCurrentPlanIndex == m_lControlPlans.end() ){
                //or if we have overshot all plans, clear
                while(m_lControlPlans.begin() != m_lControlPlans.end() )
                {
                    delete(m_lControlPlans.front());
                    m_lControlPlans.erase(m_lControlPlans.begin());
                }
            }else{
                if(nCurrentPlanIndex != m_lControlPlans.begin() ){
                    //remove all plans before the current plan
                    while(m_lControlPlans.begin() != nCurrentPlanIndex) {
                        //delete this plan
                        delete(m_lControlPlans.front());
                        m_lControlPlans.erase(m_lControlPlans.begin());
                    }
                    //the active plan should now be the first plan
                    nCurrentPlanIndex = m_lControlPlans.begin();
                }
            }
        }


        VehicleState currentState;
        {
            std::unique_lock<std::mutex> lock(m_PoseMutex);
            currentState = m_CurrentState;
        }

//        ApplyVelocitesFunctor5d compDelayFunctor(m_pModel,planStartTorques, &m_lCurrentCommands);
//        //compDelayFunctor.ResetPreviousCommands();
//        compDelayFunctor.SetNoDelay(false);

//        //push the state forward by the duration of the solver if we have commmands
//        MotionSample compDelaySample;
//        double commandTime = dPlanStartTime;
//        double maxTime =  dPlanStartTime + (m_dMaxControlPlanTime*m_dLookaheadTime);
//        while(commandTime < maxTime){
//            GetCurrentCommands(commandTime,m_LastCommand);
//            m_LastCommand.m_dT = std::min(m_dt,maxTime - commandTime);
//            m_lCurrentCommands.insert(m_lCurrentCommands.begin(),m_LastCommand);
//            compDelaySample.m_vCommands.push_back(m_LastCommand);
//            commandTime += m_dt;
//        }

//        if(compDelaySample.m_vCommands.size() > 0){
//            compDelayFunctor.ApplyVelocities(currentState,compDelaySample,0,true);
//            currentState = compDelaySample.m_vStates.back();
//        }

//        //also push forward the start time of this plan
//        dPlanStartTime += (m_dMaxControlPlanTime*m_dLookaheadTime);

        ApplyVelocitesFunctor5d delayFunctor(m_pModel,planStartTorques, NULL);
        //push forward the start state if there are commands stacked up
        MotionSample delaySample;
        double totalDelay = delayFunctor.GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
        if(totalDelay > 0 && m_lCurrentCommands.size() != 0){
            for(const ControlCommand& command: m_lCurrentCommands){
                if(totalDelay <= 0){
                    break;
                }
                ControlCommand delayCommand = command;
                delayCommand.m_dT = std::min(totalDelay,command.m_dT);
                delaySample.m_vCommands.insert(delaySample.m_vCommands.begin(),delayCommand);
                totalDelay -= delayCommand.m_dT;
            }
            //delayFunctor.ResetPreviousCommands();
            delayFunctor.SetNoDelay(true);
            //this applyvelocities call has noCompensation set to true, as the commands
            //are from a previous plan which includes compensation
            delayFunctor.ApplyVelocities(currentState,delaySample,0,true);
            //and now set the starting state to this new value
            pPlan->m_StartState = delaySample.m_vStates.back();
            m_LastCommand = delaySample.m_vCommands.back();
        }else{
            Eigen::Vector3d targetVel;
            Sophus::SE3d targetPos;
            GetCurrentCommands(dPlanStartTime,m_LastCommand,targetVel,targetPos);
            pPlan->m_StartState = currentState;
        }

        planStartTorques = m_LastCommand.m_dTorque;
        planStartCurvature = m_LastCommand.m_dCurvature;

        //dout("Plan starting curvature: " << planStartCurvature);

        //double distanceToPath = 0;
        //if we do not have a plan, create new one from our
        //current position
        if(m_lControlPlans.empty()){
            //get the starting curvature of our current plan

            //set the start time as now
            pPlan->m_dStartTime = dPlanStartTime;
            pPlan->m_nStartSegmentIndex = oldPlanStartSegment;
            pPlan->m_nStartSampleIndex = oldPlanStartSample;

            //start by finding the closest segment to our current location
            if(m_bFirstPose){
                //if this is the first pose, search everywhere for the car
                oldPlanStartSegment = 0;
                oldPlanStartSample = 0;
                sampleCount = 0;
                for(size_t jj = 0 ; jj < m_vSegmentSamples.size() ; jj++) {
                    sampleCount += m_vSegmentSamples[jj].m_vCommands.size();
                }
                m_bFirstPose = false;
                AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex,0,sampleCount);
            }else{
               AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex);
            }

        }else {
            if(nCurrentSampleIndex == -1) {
                //if we have overshot the current plan, function must be called again to create a new plan
                dout("Overshot plan.");
                return false;
            }else {
                //get the curvature at the end of the projection to have a smooth transition in steering
                pPlan->m_dStartTime = dPlanStartTime;

                pPlan->m_nStartSegmentIndex = (*nCurrentPlanIndex)->m_nStartSegmentIndex;
                //push forward the index by the precalculated amount
                pPlan->m_nStartSampleIndex = (*nCurrentPlanIndex)->m_nStartSampleIndex;// + nCurrentSampleIndex;
                MotionSample::FixSampleIndexOverflow(m_vSegmentSamples,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex);

                AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex);
            }
        }

        if(g_bForceZeroStartingCurvature == true){
            planStartCurvature = 0;
        }
        pPlan->m_StartState.m_dCurvature = planStartCurvature;

        MotionSample trajectorySample;
        VelocityProfile profile;
        //prepare the trajectory ahead
        CarController::PrepareLookaheadTrajectory(m_vSegmentSamples,pPlan,profile,trajectorySample,g_dInitialLookaheadTime);

        ApplyVelocitesFunctor5d functor(m_pModel,planStartTorques, NULL);
        functor.SetNoDelay(true);
        LocalProblem problem(&functor,pPlan->m_StartState,pPlan->m_GoalState,m_dt);
        problem.m_dStartTorques = planStartTorques;
        problem.m_CurrentSolution.m_dMinTrajectoryTime = g_dInitialLookaheadTime;
        problem.m_vVelProfile = profile;


        //solve the control plan
        if( _SolveControlPlan(pPlan,problem,trajectorySample) == false ) {
            //do not use the plan
            dout("Could not solve plan.");
            return false;
        }



        //only need to sample the planner if the plan is not airborne
        if( _SampleControlPlan(pPlan,problem) == false ) {
            dout("Failed to sample plan.");
            return false;
        }

        //dout("Plan start heading is " << pPlan->m_StartState.GetTheta() << " and goal heading is " << pPlan->m_GoalState.GetTheta() <<
        //     " and traj end heading is " << pPlan->m_Sample.m_vStates.back().GetTheta());

        //double controlDelay = problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
        double newLookahead = std::max(std::min(problem.m_pBestSolution->m_dMinTrajectoryTime, g_dMaxLookaheadTime),g_dMinLookaheadTime);
        m_dLookaheadTime = g_dLookaheadEmaWeight*newLookahead + (1-g_dLookaheadEmaWeight)*m_dLookaheadTime;

        //dout("Planned control with norm " << problem.m_dCurrentNorm << " and starting curvature " << pPlan->m_StartState.m_dCurvature);
        pPlan->m_dStartPose = pPlan->m_StartState.m_dTwv;
        pPlan->m_dEndPose = pPlan->m_GoalState.m_dTwv;


        {
            std::unique_lock<std::mutex> lock(m_PlanMutex);
            pPlan->m_nPlanId = rand() % 10000;
            //dout("Created control plan id:" << pPlan->m_nPlanId << " with starting torques: " << planStartTorques.transpose() << "with norm " << m_pPlanner->GetCurrentNorm());
            m_lControlPlans.push_back(pPlan);
        }

        //update the old plan segment and samples
        oldPlanStartSegment = pPlan->m_nStartSegmentIndex;
        oldPlanStartSample = pPlan->m_nStartSampleIndex;

        //make sure the pointer returned back is valid
        pPlanOut = pPlan;

        //do this so we create a new plan in the next iteration
        pPlan = NULL;


    }catch(...)
    {
        dout("Exception caught while planning.");
        return false;
    }
    return true;
}

VehicleState CarController::GetCurrentPose() {
    VehicleState poseOut;
    std::unique_lock<std::mutex> lock(m_PoseMutex);
    poseOut = m_CurrentState;
    return poseOut;
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::SetCurrentPoseFromCarModel(BulletCarModel* pModel, int nWorldId) {
    std::unique_lock<std::mutex> lock(m_PoseMutex);
    //Sophus::SE3d oldTwv = m_CurrentState.m_dTwv;
    pModel->GetVehicleState(0,m_CurrentState);
    //remove the car offset from the car state
    //m_CurrentState.m_dTwv.block<3,1>(0,3) += m_CurrentState.m_dTwv.block<3,1>(0,2)*CAR_HEIGHT_OFFSET;
    pModel->GetCommandHistory(0,m_lCurrentCommands);
    m_bPoseUpdated = g_bFreezeControl ? false : true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::SetCurrentPose(VehicleState pose, CommandList* pCommandList /*= NULL*/) {
    std::unique_lock<std::mutex> lock(m_PoseMutex);

    if( std::isfinite(pose.m_dV[0]) == false ){
        assert(false);
    }

    m_CurrentState = pose;

    if(pCommandList != NULL) {
        m_lCurrentCommands = *pCommandList;
    }

    m_bPoseUpdated = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
double CarController::GetLastPlanStartTime()
{
    std::unique_lock<std::mutex> lock(m_PlanMutex);
    if(m_lControlPlans.empty() == false){
        return m_lControlPlans.back()->m_dStartTime;
    }else{
        return -1;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void CarController::GetCurrentCommands(const double time,
                                       ControlCommand& command)
{
    Eigen::Vector3d targetVel;
    Sophus::SE3d dT_target;
    GetCurrentCommands(time,command,targetVel,dT_target);
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::GetCurrentCommands(const double time,
                                       ControlCommand& command,
                                       Eigen::Vector3d& targetVel,
                                       Sophus::SE3d& dT_target)
{
    std::unique_lock<std::mutex> lock(m_PlanMutex);
    int nCurrentSampleIndex;
    PlanPtrList::iterator nCurrentPlanIndex;
    double interpolationAmount;
    _GetCurrentPlanIndex(time,nCurrentPlanIndex,nCurrentSampleIndex,interpolationAmount);
    if( nCurrentSampleIndex == -1 || nCurrentPlanIndex == m_lControlPlans.end() ) {
        //dout("GetCurrentCommands returning last commands a:" << m_dLastAccel << " c:" << m_dLastTurnRate << " t:" << m_dLastTorques.transpose());
        command.m_dForce = m_pModel->GetParameters(0)[CarParameters::AccelOffset]*SERVO_RANGE;
        command.m_dPhi = m_pModel->GetParameters(0)[CarParameters::SteeringOffset]*SERVO_RANGE;
        command.m_dTorque = Eigen::Vector3d::Zero();//m_dLastTorques;
        //dout("Torque output of: [ " << torques.transpose() << "] from previous plan");
    }else {
        command.m_dForce = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dForce +
                            interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dForce;

        command.m_dPhi =   (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dPhi +
                           interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dPhi;


        command.m_dCurvature = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dCurvature +
                                interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dCurvature;

        command.m_dTorque = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dTorque +
                            interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dTorque;


        //dout("v: " << m_vSegmentSamples[(*nCurrentPlanIndex)->m_nStartSegmentIndex].m_vStates[(*nCurrentPlanIndex)->m_nStartSampleIndex].m_dV.transpose());
        //calculate target values

        int currentSegIndex, currentSampleIndex;
        currentSegIndex = (*nCurrentPlanIndex)->m_nStartSegmentIndex;
        currentSampleIndex = (*nCurrentPlanIndex)->m_nStartSampleIndex + nCurrentSampleIndex;
        MotionSample::FixSampleIndexOverflow(m_vSegmentSamples,currentSegIndex,currentSampleIndex);
        dT_target =  m_vSegmentSamples[currentSegIndex].m_vStates[currentSampleIndex].m_dTwv;
        targetVel = m_vSegmentSamples[currentSegIndex].m_vStates[currentSampleIndex].m_dV;

        //dout("GetCurrentCommands planid:" << (*nCurrentPlanIndex)->m_nPlanId << " sample index:" << nCurrentSampleIndex << " returning interpolation with i:" << interpolationAmount << " a:" << accel << " c:" << curvature << " t:" << torques.transpose());

        m_LastCommand.m_dForce = command.m_dForce;
        m_LastCommand.m_dCurvature = command.m_dCurvature;
        m_LastCommand.m_dPhi = command.m_dPhi;
        m_LastCommand.m_dTorque = command.m_dTorque;
    }


}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::_GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sampleIndex, double& interpolationAmount) {
    interpolationAmount = 0;
    sampleIndex = -1;
    planIndex = m_lControlPlans.end();
    bool bPlanValid = false;

    if(m_lControlPlans.empty() == false) {
        //for(int ii = 0; ii < m_vControlPlans.size() ; ii++) {
        for(PlanPtrList::iterator it = m_lControlPlans.begin() ; it != m_lControlPlans.end() ; it++) {
            sampleIndex = 0;

            //only if the current time is within the bounds of the plan, will we go and search
            //for the exact command
            if(currentTime >= (*it)->m_Sample.m_vCommands.front().m_dTime &&
               currentTime <= (*it)->m_Sample.m_vCommands.back().m_dTime &&
               (*it)->m_Sample.m_vCommands.size() > 1){
                planIndex = it;
                for(size_t jj = 1; jj < (*it)->m_Sample.m_vCommands.size() ; jj++){
                    if(((*it)->m_Sample.m_vCommands[jj].m_dTime) >= currentTime){
                        bPlanValid = true; //the plan has not yet finished
                        if(sampleIndex != -1){
                            double prevTime = (*it)->m_Sample.m_vCommands[sampleIndex].m_dTime;
                            double nextTime = (*it)->m_Sample.m_vCommands[jj].m_dTime;
                            interpolationAmount = (currentTime - prevTime) /(nextTime-prevTime);
                        }
                        break;
                    }
                    sampleIndex = jj;
                }
            }
        }
    }

    if( bPlanValid == false ) {
        planIndex = m_lControlPlans.end();
        sampleIndex = -1;
    }

    if( sampleIndex != m_nLastCurrentPlanIndex) {
        m_nLastCurrentPlanIndex = sampleIndex;
    }

}




////////////////////////////////////////////////////////////////
double CarController::AdjustStartingSample(const std::vector<MotionSample>& segmentSamples,
                                           VehicleState& state,
                                           int& segmentIndex,
                                           int& sampleIndex,
                                           int lowerLimit /*= 100*/,
                                           int upperLimit /*= 100*/)
{
    //move within a certain neighbourhood of the samples and see if you can find a minimum distance to the trajectory
    int currentSegmentIndex = segmentIndex;
    int currentSampleIndex = sampleIndex;
    double minDistance = DBL_MAX;
    const VehicleState& currentState = segmentSamples[currentSegmentIndex].m_vStates[currentSampleIndex];
    Eigen::Vector3d distVect = currentState.m_dTwv.translation() - state.m_dTwv.translation();

    int minSegmentIndex = segmentIndex;
    int minSampleIndex = sampleIndex;

    //offset by a certain amount before
    currentSampleIndex -= lowerLimit; //0.5 seconds
    MotionSample::FixSampleIndexOverflow(segmentSamples,currentSegmentIndex,currentSampleIndex);

    for(int ii = 0; ii < upperLimit + lowerLimit ; ii++){ //-0.5s -> +0.5s
        //fix any over/underflow
        MotionSample::FixSampleIndexOverflow(segmentSamples,currentSegmentIndex,currentSampleIndex);

        //see if this distance is less than the prevous
        const VehicleState& currentState2 = segmentSamples[currentSegmentIndex].m_vStates[currentSampleIndex];

        distVect = currentState2.m_dTwv.translation() - state.m_dTwv.translation();
        double sn = distVect.squaredNorm();
        if( sn <= minDistance ) {
            minDistance = sn;
            minSegmentIndex = currentSegmentIndex;
            minSampleIndex = currentSampleIndex;
        }

        //increment the current sample
        currentSampleIndex++;
    }
    sampleIndex = minSampleIndex;
    segmentIndex = minSegmentIndex;

    //return the minimum distance
    return minDistance;
}

////////////////////////////////////////////////////////////////
void CarController::PrepareLookaheadTrajectory(const std::vector<MotionSample> &vSegmentSamples,
                                               ControlPlan *pPlan,
                                               VelocityProfile& trajectoryProfile,
                                               MotionSample& trajectorySample,
                                               const double dLookaheadTime)
{
    double dLookahead = dLookaheadTime;
    //create a motion sample from this plan
    trajectoryProfile.push_back(VelocityProfileNode(0,pPlan->m_StartState.m_dV.norm()));

    int seg = pPlan->m_nStartSegmentIndex;
    int spl = pPlan->m_nStartSampleIndex;
    //reserve some states to improve efficiency
    trajectorySample.m_vStates.reserve(vSegmentSamples[seg].m_vStates.size());
    double trajTime = 0;
    while(trajTime <= dLookahead){
        trajectorySample.m_vStates.push_back(vSegmentSamples[seg].m_vStates[spl]);
        //set the correct time on the trajectory and increment
        trajectorySample.m_vStates.back().m_dTime = trajTime;
        trajTime += vSegmentSamples[seg].m_vCommands[spl].m_dT;
        spl++;
        if(MotionSample::FixSampleIndexOverflow(vSegmentSamples,seg,spl) && trajTime >= g_dMinLookaheadTime){
            trajectoryProfile.push_back(VelocityProfileNode(trajectorySample.GetDistance(),trajectorySample.m_vStates.back().m_dV.norm()));
        }

//        if(trajectorySample.m_vStates.back().IsAirborne()){
//            dLookahead = std::max(dLookahead,trajTime);
//        }
    }

    const double totalDist = trajectorySample.GetDistance();
    trajectoryProfile.push_back(VelocityProfileNode(totalDist,trajectorySample.m_vStates.back().m_dV.norm()));
    for(VelocityProfileNode& node : trajectoryProfile){
        node.m_dDistanceRatio /= totalDist;
    }

    const int nTotalTrajSamples = trajectorySample.m_vStates.size();
    //now add some more states to the end of the sample (for trajectory tracking)
    for(int jj = 0 ; jj < nTotalTrajSamples ; jj++){
        trajectorySample.m_vStates.push_back(vSegmentSamples[seg].m_vStates[spl]);
        //set the correct time on the trajectory and increment
        trajectorySample.m_vStates.back().m_dTime = trajTime;
        trajTime += vSegmentSamples[seg].m_vCommands[spl].m_dT;
        spl++;
        MotionSample::FixSampleIndexOverflow(vSegmentSamples,seg,spl);
    }



    pPlan->m_nEndSegmentIndex = pPlan->m_nStartSegmentIndex;
    pPlan->m_nEndSampleIndex = pPlan->m_nStartSampleIndex+nTotalTrajSamples;
    MotionSample::FixSampleIndexOverflow(vSegmentSamples,pPlan->m_nEndSegmentIndex,pPlan->m_nEndSampleIndex);
    pPlan->m_GoalState = vSegmentSamples[pPlan->m_nEndSegmentIndex].m_vStates[pPlan->m_nEndSampleIndex];
    //pPlan->m_GoalState.m_dV = pPlan->m_StartState.m_dV;

    //search for an airborne transition and adjust weights accordingly
//            int searchSeg = pPlan->m_nStartSegmentIndex;
//            int searchSpl = pPlan->m_nStartSampleIndex+lookaheadSamples;
//            MotionSample::FixSampleIndexOverflow(segmentSamples,searchSeg,searchSpl);
//            double totalTransitionSearchTime = 0.3;
//            double transitionTime = -1;
//            if(segmentSamples[searchSeg].m_vStates[searchSpl].IsAirborne() == false){
//                while(totalTransitionSearchTime > 0){
//                    searchSpl++;
//                    MotionSample::FixSampleIndexOverflow(segmentSamples,searchSeg,searchSpl);
//                    totalTransitionSearchTime -= segmentSamples[searchSeg].m_vCommands[searchSpl].m_dT;
//                    if(segmentSamples[searchSeg].m_vStates[searchSpl].IsAirborne() && transitionTime == -1){
//                        transitionTime = totalTransitionSearchTime;
//                        break;
//                    }
//                }
//            }

//            //if there is a transition from ground to airborne in this sample, then we must modify the weights
//            if(transitionTime == -1){
//                m_pPlanner->m_dTrajWeight(3) = THETA_MULTIPLIER;
//            }else{
//                interpolationAmount = transitionTime/totalTransitionSearchTime;
//                m_pPlanner->m_dTrajWeight(3) = THETA_MULTIPLIER*4*(interpolationAmount) + THETA_MULTIPLIER*(1-interpolationAmount);
//                dout("Transition in " << transitionTime << "s Setting theta weight to " << m_pPlanner->m_dTrajWeight(3) );
//            }



    //pPlan->m_StartState.m_dCurvature = planStartCurvature;

    if(pPlan->m_GoalState.IsAirborne()){
        pPlan->m_GoalState.AlignWithVelocityVector();
        pPlan->m_GoalState.m_dCurvature = 0;
    }else{
        pPlan->m_GoalState.m_dCurvature = vSegmentSamples[pPlan->m_nEndSegmentIndex].m_vCommands[pPlan->m_nEndSampleIndex].m_dCurvature;
    }

    if(pPlan->m_StartState.m_dV.norm() >= 0.5){
        pPlan->m_StartState.AlignWithVelocityVector();
    }

}

