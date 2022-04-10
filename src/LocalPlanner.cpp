#include <CarPlanner/CVarHelpers.h>
#include <CarPlanner/LocalPlanner.h>

static bool& g_bUseCentralDifferences = CVarUtils::CreateGetUnsavedCVar("debug.UseCentralDifferences",true);
static double& g_dSuccessNorm = CVarUtils::CreateGetUnsavedCVar("debug.SuccessNorm",0.01);
static double& g_dTimeTarget = CVarUtils::CreateGetUnsavedCVar("debug.TimeTarget",0.00);
//static bool& g_bUseGoalPoseStepping = CVarUtils::CreateGetCVar("debug.UseGoalPoseStepping",false);
static bool& g_bDisableDamping = CVarUtils::CreateGetUnsavedCVar("debug.DisableDamping",false);
static bool& g_bMonotonicCost(CVarUtils::CreateGetUnsavedCVar("debug.MonotonicCost", true,""));
static bool& g_bVerbose(CVarUtils::CreateGetUnsavedCVar("debug.Verbose", false,""));
static bool& g_bTrajectoryCost(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCost", true,""));
static int& g_nTrajectoryCostSegments(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCostSegments", 10,""));

struct ApplyCommandsThreadFunctor {
    ApplyCommandsThreadFunctor(LocalPlanner *pPlanner, LocalProblem& problem,const int index , Eigen::Vector6d& poseOut,Eigen::VectorXd& errorOut,
                               MotionSample& sample, const bool bSolveBoundary = false) :
        m_pPlanner(pPlanner),
        m_Problem(problem),
        m_index(index),
        m_poseOut(poseOut),
        m_dErrorOut(errorOut),
        m_Sample(sample),
        m_bSolveBoundary(bSolveBoundary)

    {}

    void operator()()
    {
        SetThreadName((boost::format("Bullet Simulation Thread #%d") % m_index).str().c_str());
        if(m_bSolveBoundary){
            m_Problem.m_pBoundarySovler->Solve(&m_Problem.m_BoundaryProblem);
        }
        m_poseOut = m_pPlanner->SimulateTrajectory(m_Sample,m_Problem, m_index);
        m_dErrorOut = m_pPlanner->_CalculateSampleError(m_Sample,m_Problem,m_Problem.m_CurrentSolution.m_dMinTrajectoryTime);
    }

    LocalPlanner *m_pPlanner;
    //const double m_startingCurvature;
    //const double m_dt;
    LocalProblem& m_Problem;
    const int m_index;    
    Eigen::Vector6d& m_poseOut;
    Eigen::VectorXd& m_dErrorOut;
    MotionSample& m_Sample;
    bool m_bSolveBoundary;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1,const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double& dInterpolationFactor)
{
    Eigen::Vector3d vAB = (line2.head(3) - line1.head(3));
    Eigen::Vector3d vAC = (point.head(3) - line1.head(3));
    double dAB = vAB.norm();
    double dProjection = vAC.dot(vAB.normalized());
//    if(dProjection < 0){
//        DLOG(INFO) << "Negative vector projection of " << dProjection << " when calculating point line error...";
//    }
    dInterpolationFactor = std::max(std::min(dProjection/dAB,1.0),0.0);

//    if(interpolation > 1.0 || interpolation < 0){
//        DLOG(INFO) << "Point/line interpolation factor out of bounds at " << interpolation << "Interpolating between [" << line1.head(3).transpose() << "], [" << line2.head(3).transpose() << "] and [" << point.head(3).transpose();
//    }
    //now calculate the interpolated value
    Eigen::Vector6d intVal = (1.0-dInterpolationFactor)*line1 + (dInterpolationFactor)*line2;
    intVal[3] = rpg::AngleWrap(intVal[3]);

    //and now we can calculate the error to the interpolated pose
    Eigen::VectorXd intError(TRAJ_UNIT_ERROR_TERMS);
    intError.head(3) = intVal.head(3) - point.head(3);
    intError[3] = rpg::AngleWrap(intVal[3] - point[3]);
    intError[4] = intVal[5] - point[5];
    return intError;
}

LocalPlanner::LocalPlanner() :
    m_dEps(CVarUtils::CreateUnsavedCVar("planner.Epsilon", 1e-6, "Epsilon value used in finite differences.")),
    m_dPointWeight(CVarUtils::CreateUnsavedCVar("planner.PointCostWeights",Eigen::MatrixXd(1,1))),
    m_dTrajWeight(CVarUtils::CreateUnsavedCVar("planner.TrajCostWeights",Eigen::MatrixXd(1,1))),
    m_nPlanCounter(0)
{
    m_ThreadPool.size_controller().resize(8);

    //weight matrix
    m_dPointWeight = Eigen::MatrixXd(POINT_COST_ERROR_TERMS,1);
    m_dTrajWeight = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
    m_dPointWeight.setIdentity();
    m_dTrajWeight.setIdentity();

    m_dPointWeight(0) = XYZ_WEIGHT;
    m_dPointWeight(1) = XYZ_WEIGHT;
    m_dPointWeight(2) = XYZ_WEIGHT;
    m_dPointWeight(3) = THETA_WEIGHT;
    m_dPointWeight(4) = VEL_WEIGHT_POINT;
    //m_dPointWeight(5) = CURV_WEIGHT;
    m_dPointWeight(5) = TILT_WEIGHT_POINT;
    m_dPointWeight(6) = CONTACT_WEIGHT_POINT;

    m_dTrajWeight(0) = XYZ_WEIGHT;
    m_dTrajWeight(1) = XYZ_WEIGHT;
    m_dTrajWeight(2) = XYZ_WEIGHT;
    m_dTrajWeight(3) = THETA_WEIGHT;
    m_dTrajWeight(4) = VEL_WEIGHT_TRAJ;
    m_dTrajWeight(5) = TIME_WEIGHT;
    m_dTrajWeight(6) = CURV_WEIGHT;
    //m_dTrajWeight(7) = BADNESS_WEIGHT;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::SamplePath(const LocalProblem& problem, Eigen::Vector3dAlignedVec& vSamples, bool bBestSolution /* = true */ )
{
    vSamples.clear();
    BezierBoundaryProblem boundaryProblem = problem.m_BoundaryProblem;
    Sophus::SE2d T_start(Sophus::SO2(problem.m_dStartPose[3]),problem.m_dStartPose.head(2));
    vSamples.reserve(boundaryProblem.m_vPts.size());
    for(const Eigen::Vector2d& pt : boundaryProblem.m_vPts) {
        Eigen::Vector3d dPos(pt[0],pt[1],0);
        dPos.head(2) = T_start*dPos.head(2);

        //now transform this into the proper 3d pose
        dPos = problem.m_dT3dInv * dPos;
        vSamples.push_back(dPos);
    }
}

///////////////////////////////////////////////////////////////////////
Eigen::Vector6d LocalPlanner::_Transform3dGoalPose(const VehicleState& state, const LocalProblem &problem) const
{
    //also transfer this pose into the 3d projected space we are working on
    const Sophus::SE3d dPose = problem.m_dT3d * state.m_dTwv;
    Eigen::Vector6d untransformedPose;
    untransformedPose << dPose.translation()[0],
                         dPose.translation()[1],
                         dPose.translation()[2],
                         atan2( dPose.matrix()(1,0), dPose.matrix()(0,0)),
                         0,
                         state.m_dV.norm();

    return _TransformGoalPose(untransformedPose,problem);
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d  LocalPlanner::_TransformGoalPose(const Eigen::Vector6d& dGoalPose,const LocalProblem& problem) const
{
    Eigen::Vector6d result;
    Eigen::Vector3d pt;
    pt << dGoalPose[0], dGoalPose[1], 1;
    pt = problem.m_dTinv * pt;
    result << pt[0], pt[1], dGoalPose[2], rpg::AngleWrap( dGoalPose[3] - problem.m_dStartPose[3] ), dGoalPose[4], dGoalPose[5];
    return result;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_GetTrajectoryError(const MotionSample& sample,
                                                  const Eigen::Vector6dAlignedVec& vTransformedPoses,
                                                  const Eigen::Vector6d& endPose,
                                                  double& dMinTime) const
{
    Eigen::VectorXd error(TRAJ_UNIT_ERROR_TERMS);
    int nTrajSize = sample.m_vStates.size();
    Eigen::Vector6d minPose, minPoseBefore, minPoseAfter;
    int nMinIdx = -1;
    //now find the closes point in the trajectory to our current position
    double dMinDist = DBL_MAX;
    for(int ii = 0; ii < nTrajSize ; ii++){
        //find the closest point to our current location
        Eigen::Vector3d cartError = vTransformedPoses[ii].head(3) - endPose.head(3);
        double norm = (cartError).norm();
        if(norm < dMinDist){
            dMinTime = sample.m_vStates[ii].m_dTime;
            minPose = vTransformedPoses[ii];;
            minPoseAfter = ii < (nTrajSize-1) ? vTransformedPoses[ii+1] : minPose;
            minPoseBefore = ii > 0 ? vTransformedPoses[ii-1] : minPose;
            nMinIdx = ii;
            dMinDist = norm;
        }
    }

    //reset the error
    error.setZero();

    //calculate the distance at the minimum
    error.head(3) = minPose.head(3) - endPose.head(3);
    error[3] = rpg::AngleWrap(minPose[3] - endPose[3]);
    error[4] = minPose[5] - endPose[5];

    //now calculate the distance on both sides and find the minimum
    double dInterpolationFactor;
    Eigen::VectorXd beforeError;
    if(minPose != minPoseBefore){
        beforeError = GetPointLineError(minPoseBefore,minPose,endPose,dInterpolationFactor);
        if(beforeError.head(3).norm() < error.head(3).norm()){
            dMinTime = (1.0-dInterpolationFactor)*sample.m_vStates[nMinIdx-1].m_dTime + dInterpolationFactor*sample.m_vStates[nMinIdx].m_dTime;
            error = beforeError;
        }
    }


    Eigen::VectorXd afterError;
    if(minPose != minPoseAfter){
        afterError = GetPointLineError(minPose,minPoseAfter,endPose,dInterpolationFactor);
        if(afterError.head(3).norm() < error.head(3).norm()){
            dMinTime = (1.0-dInterpolationFactor)*sample.m_vStates[nMinIdx].m_dTime + dInterpolationFactor*sample.m_vStates[nMinIdx+1].m_dTime;
            error = afterError;
        }
    }
    //error[4] = minPose[5] - endPose[5];
//    for(int ii = 0; ii < error.rows() ; ii++){
//        error[ii] = fabs(error[ii]);
//    }
    return error;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_GetWeightVector(const LocalProblem& problem)
{
    int errorVecSize = problem.m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd dW;
    Eigen::VectorXd trajW(errorVecSize);
    if(problem.m_eCostMode == eCostTrajectory){
        double dFactor = 1.0/((double)g_nTrajectoryCostSegments);
        for(int ii = 0; ii < g_nTrajectoryCostSegments ; ii++){
            trajW.segment(ii*TRAJ_UNIT_ERROR_TERMS,TRAJ_UNIT_ERROR_TERMS) = m_dTrajWeight.col(0).head(TRAJ_UNIT_ERROR_TERMS)*dFactor;
            dFactor *= 2;
        }
        trajW.tail(TRAJ_EXTRA_ERROR_TERMS) = m_dTrajWeight.col(0).tail(TRAJ_EXTRA_ERROR_TERMS);
        dW = trajW;
        //DLOG(INFO) << "Trjaectory weights are: " << trajW.transpose();
    }else{
        dW = m_dPointWeight.col(0).head(POINT_COST_ERROR_TERMS);
    }
    return dW;
}

///////////////////////////////////////////////////////////////////////
double LocalPlanner::_CalculateErrorNorm(const LocalProblem& problem,const Eigen::VectorXd& dError)
{
    Eigen::VectorXd dW = _GetWeightVector(problem);
    Eigen::VectorXd error = dError;
    //DLOG(INFO) << "error vector is " << error.transpose();
    error.array() *= dW.array();
    return error.norm();
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_CalculateSampleError(const MotionSample& sample, LocalProblem& problem, double& dMinTrajTime) const
{
    int errorVecSize = problem.m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd error;
    if(sample.m_vStates.size() == 0 ){
        DLOG(ERROR) << problem.m_nPlanId << ":Sample with size 0 detected. Aborting.";
        error = Eigen::VectorXd(errorVecSize);
        error.setOnes();
        error *= DBL_MAX;
        return error;
    }
    //get the normalized velocity
    VehicleState state = sample.m_vStates.back();
    Eigen::Vector3d dW_goal = problem.m_GoalState.m_dTwv.so3().inverse()* state.m_dW;
    //double dPrevAngle = state.GetTheta();
    if(state.IsAirborne()){
        state.AlignWithVelocityVector();
        //DLOG(INFO) << "End state transformed from " << dPrevAngle << " to " << state.GetTheta();
    }

    Eigen::Vector6d endPose = _Transform3dGoalPose(state,problem);
    if(problem.m_eCostMode == eCostPoint){
        error =Eigen::VectorXd(errorVecSize);
        //Eigen::Vector3d cartError = T2Cart_2D(Cart2T_2D(problem.m_dTransformedGoal.head(3))*Tinv_2D(Cart2T_2D(endPose.head(3))));
        //error.head(3) = cartError;
        //transform the angular velocity into the plane of goal pose

        error.head(3) = problem.m_dTransformedGoal.head(3) - endPose.head(3);
        error[3] = rpg::AngleWrap(problem.m_dTransformedGoal[3] - endPose[3]);
        error[4] = problem.m_dTransformedGoal[5] - endPose[5];
        //error[5] = state.m_dV.norm()*dW_goal[2] - problem.m_GoalState.m_dCurvature;

        //error[5] = sample.GetBadnessCost();
        //error[5] = -std::log(problem.m_BoundaryProblem.m_dAggressiveness);
        //error.array() *= m_dPointWeight.block<POINT_COST_ERROR_TERMS,1>(0,0).array();
        //DLOG(INFO) << "Error vector is " << error.transpose() << " weights are " << m_dPointWeight.transpose();

        error[5] = sample.GetTiltCost();
        error[6] = sample.GetContactCost();
    }else if(problem.m_eCostMode == eCostTrajectory){
        error =Eigen::VectorXd(errorVecSize);

        if(problem.m_vTransformedTrajectory.empty()){
            int nTrajSize = problem.m_Trajectory.m_vStates.size();
            problem.m_vTransformedTrajectory.reserve(nTrajSize);
            //push back the transformed poses
            for(int ii = 0; ii < nTrajSize ; ii++){
                VehicleState tempState = problem.m_Trajectory.m_vStates[ii];
                if(tempState.IsAirborne()){
                    tempState.AlignWithVelocityVector();
                }
                problem.m_vTransformedTrajectory.push_back(_Transform3dGoalPose(tempState, problem));
            }
        }


        error.setZero();
        error.segment(TRAJ_UNIT_ERROR_TERMS*(g_nTrajectoryCostSegments-1),TRAJ_UNIT_ERROR_TERMS)  = _GetTrajectoryError(problem.m_Trajectory,
                                                                                                                         problem.m_vTransformedTrajectory,
                                                                                                                         endPose,
                                                                                                                         dMinTrajTime);

        //now that we have the minimum point, calculate trajectory error points along the way
        int counter = 0;
        int nStartIndex = 0;

        for(double ii = dMinTrajTime/(double)g_nTrajectoryCostSegments ;
                   ii < dMinTrajTime && counter < ((double)g_nTrajectoryCostSegments-1) ;
                   ii += dMinTrajTime/(double)g_nTrajectoryCostSegments ){
            //get the poses at this point in time
            //Eigen::Vector6d poseTraj = _Transform3dGoalPose(VehicleState::GetInterpolatedState(problem.m_Trajectory.m_vStates,nStartIndex,ii,nStartIndex),problem);
            Eigen::Vector6d poseSample = _Transform3dGoalPose(VehicleState::GetInterpolatedState(sample.m_vStates,nStartIndex,ii,nStartIndex),problem);

            double dMinTime;
            error.segment(TRAJ_UNIT_ERROR_TERMS*counter,TRAJ_UNIT_ERROR_TERMS) = _GetTrajectoryError(problem.m_Trajectory,
                                                                                                     problem.m_vTransformedTrajectory,
                                                                                                     poseSample,
                                                                                                    dMinTime);
            counter++;
        }
        error /= (double)g_nTrajectoryCostSegments;


        //segment cost

        error[error.rows()-2] = g_dTimeTarget-dMinTrajTime;
        error[error.rows()-1] = state.m_dV.norm()*dW_goal[2] - problem.m_GoalState.m_dCurvature; // sample.GetBadnessCost();

        //error[7] = sample.GetBadnessCost();
        //error[6] = -std::log(problem.m_BoundaryProblem.m_dAggressiveness);
        //error.array() *= m_dTrajWeight.block<TRAJ_COST_ERROR_TERMS,1>(0,0).array();
    }
    return error;
}

///////////////////////////////////////////////////////////////////////
bool LocalPlanner::_CalculateJacobian(LocalProblem& problem,
                                      Eigen::VectorXd& dCurrentErrorVec,
                                      LocalProblemSolution& coordinateDescent,
                                      Eigen::MatrixXd& J)
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd errors[OPT_DIM*2],dCurrentError;
    std::vector<std::shared_ptr<LocalProblem > > vCubicProblems;
    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor > > vFunctors;
    vCubicProblems.resize(OPT_DIM*2);
    vFunctors.resize(OPT_DIM*2);
    Eigen::Vector6d pPoses[OPT_DIM*2],dCurrentPose;

    const double dEps = m_dEps;// * problem.m_CurrentSolution.m_dNorm;

    //g_bUseCentralDifferences = false;
    for( int ii = 0; ii < OPT_DIM; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        vCubicProblems[plusIdx] = std::make_shared<LocalProblem>(problem);
        Eigen::VectorXd delta(OPT_DIM);
        delta.setZero();
        delta(ii) += dEps;
        vCubicProblems[plusIdx]->UpdateOptParams(vCubicProblems[plusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
        vFunctors[plusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                          *vCubicProblems[plusIdx],
                                                                          plusIdx,
                                                                          pPoses[plusIdx],
                                                                          errors[plusIdx],
                                                                          (vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample),
                                                                          true);
        m_ThreadPool.schedule(*vFunctors[plusIdx]);

        if(g_bUseCentralDifferences == true){
            vCubicProblems[minusIdx] = std::make_shared<LocalProblem>(problem);
            Eigen::VectorXd delta(OPT_DIM);
            delta.setZero();
            delta(ii) -= dEps;
            vCubicProblems[minusIdx]->UpdateOptParams(vCubicProblems[minusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[minusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                               *vCubicProblems[minusIdx],
                                                                               minusIdx,
                                                                               pPoses[minusIdx],
                                                                               errors[minusIdx],
                                                                               (vCubicProblems[minusIdx]->m_CurrentSolution.m_Sample),
                                                                               true);
            m_ThreadPool.schedule(*vFunctors[minusIdx]);
        }
    }

    std::shared_ptr<LocalProblem >currentProblem = std::make_shared<LocalProblem>(problem);
    std::shared_ptr<ApplyCommandsThreadFunctor > currentFunctor =
        std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                     *currentProblem,
                                                     OPT_DIM*2+1,
                                                     dCurrentPose,
                                                     dCurrentError,
                                                     (currentProblem->m_CurrentSolution.m_Sample),
                                                     true);
    m_ThreadPool.schedule(*currentFunctor);


    //wait for all simulations to finish
    m_ThreadPool.wait();

    dCurrentErrorVec = dCurrentError;

    std::shared_ptr<LocalProblem> pCoordinateDescent;
    double dBestNorm = DBL_MAX;

    for( int ii = 0; ii < OPT_DIM ; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        double norm = _CalculateErrorNorm(*vCubicProblems[plusIdx],errors[plusIdx]);
        if(std::isfinite(norm)){
            if( norm < dBestNorm ) {
                dBestNorm = norm;
                pCoordinateDescent = (vCubicProblems[plusIdx]);
            }
        }


        if(g_bUseCentralDifferences == true){
            norm = _CalculateErrorNorm(*vCubicProblems[minusIdx],errors[minusIdx]);
            if(std::isfinite(norm)){
                if( norm < dBestNorm ) {
                    dBestNorm = norm;
                    pCoordinateDescent = (vCubicProblems[plusIdx]);
                }
            }
        }

        if(g_bVerbose){
            DLOG(INFO) << "Dimension " << ii << " norm " << norm << " error-> [" << errors[plusIdx].transpose().format(CleanFmt) << "] vs. ["  << dCurrentErrorVec.transpose().format(CleanFmt);
        }

        //now that we have all the error terms, we can set this column of the jacobians

        Eigen::VectorXd col = g_bUseCentralDifferences ? ((errors[plusIdx]) - (errors[minusIdx]))/(2.0*dEps) : ((errors[plusIdx]) - dCurrentErrorVec)/(dEps);

        J.col(ii) = -col;
        //if this term is NAN, sound the alarm
        if(std::isfinite(col[0]) == false || std::isfinite(col[1]) == false ||
           std::isfinite(col[2]) == false || std::isfinite(col[3]) == false){
            problem.m_eError = eJacobianColumnNan;
            return false;
        }

        //if this column is zero
//        if( col.norm() == 0){
//            problem.m_eError = eJacobianColumnZero;
//            return false;
//        }
    }

    coordinateDescent = pCoordinateDescent->m_CurrentSolution;
    coordinateDescent.m_dNorm = dBestNorm;


    if(g_bVerbose){
        DLOG(INFO) << "Jacobian:" << J.format(CleanFmt) << std::endl;
    }
    return true;
}


///////////////////////////////////////////////////////////////////////
double LocalPlanner::_DistanceTraveled( const double& t,const AccelerationProfile& profile ) const
{
    double totalDist = 0;
    double lastTime = 0;
    //go through each profile segment until we find the one that we're in
    for(size_t ii = 0 ; ii < profile.size() ; ii++){
        if(profile[ii].m_dEndTime < t && ii != profile.size()-1){
            totalDist += profile[ii].m_dEndDist;
            lastTime = profile[ii].m_dEndTime;
            continue;
        }else{
            double dt = t - lastTime;
            totalDist += profile[ii].m_dVStart * dt + (profile[ii].m_dVEnd - profile[ii].m_dVStart)*(dt * dt) / (2.0 * (profile[ii].m_dEndTime-lastTime));
            break;
        }
    }
    return totalDist;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::SampleAcceleration(std::vector<ControlCommand>& vCommands, LocalProblem& problem) const
{
    vCommands.clear();

    //double st;// = SegmentTime(p);
    //AccelerationProfile accelProfile;
    _GetAccelerationProfile(problem);
    //problem.m_dSegmentTime += problem.m_dSegmentTimeDelta;
    if(std::isfinite(problem.m_dSegmentTime) == false ){
        DLOG(ERROR) << problem.m_nPlanId << ":Segment time of " << problem.m_dSegmentTime << " was not finite.";
        return;
    }
    double t;
    problem.m_dSegmentTime = std::min(problem.m_dSegmentTime,problem.m_dMaxSegmentTime);
    int numSamples = (int)(problem.m_dSegmentTime/problem.m_dT + 1.0);
    vCommands.reserve(numSamples);

    size_t accelIndex = 0;
    for( t = 0; t < (problem.m_dSegmentTime) ; t+= problem.m_dT  ){
        double curvature = problem.m_pBoundarySovler->GetCurvature(&problem.m_BoundaryProblem,
                                                                   _DistanceTraveled(t,problem.m_vAccelProfile));

        double step = problem.m_dSegmentTime - t;
        double actualDt = std::min(problem.m_dT,step);

        if(problem.m_vAccelProfile[accelIndex].m_dEndTime < t){
            accelIndex++;
        }

        if(accelIndex >= problem.m_vAccelProfile.size()){
            DLOG(ERROR) << problem.m_nPlanId << ":Exceeded bounds of acceleration profile.";
            return;
        }

        //if needed, add torques
        double endTime = problem.m_dTorqueStartTime + problem.m_dTorqueDuration;
        Eigen::Vector3d dTorques = Eigen::Vector3d::Zero();
        if(t >= problem.m_dTorqueStartTime && problem.m_dTorqueStartTime != -1 && t <= (endTime)){
            dTorques(1) = problem.m_dCoefs(0) + problem.m_dCoefs(1)*(t-problem.m_dTorqueStartTime) +
                          problem.m_dCoefs(2)*powi((t-problem.m_dTorqueStartTime),2) +
                          problem.m_dCoefs(3)*powi((t-problem.m_dTorqueStartTime),3);
        }
        vCommands.push_back(ControlCommand(problem.m_vAccelProfile[accelIndex].m_dAccel+(problem.m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM]/problem.m_dSegmentTime),curvature,dTorques,actualDt,0));
    }

    //if there is control delay, add empty commands to the end, so that the control delay queue can be flushed out
//    double totalDelay = problem.m_pFunctor->GetCarModel()->GetParameters(nIndex)[CarParameters::ControlDelay];
//    if(totalDelay > 0 && problem.m_pFunctor->GetPreviousCommand().size() != 0){
//        CommandList::iterator it  = problem.m_pFunctor->GetPreviousCommand().begin();
//        while(totalDelay > 0 && it != problem.m_pFunctor->GetPreviousCommand().end()){
//            vCommands.push_back(ControlCommands(0,0,Eigen::Vector3d::Zero(),problem.m_dT,0));
//            totalDelay -= problem.m_dT;
//            ++it;
//        }
//    }
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d LocalPlanner::SimulateTrajectory(MotionSample& sample,
                                                 LocalProblem& problem,
                                                 const int nIndex /*= 0*/,
                                                 const bool& bBestSolution /* = false */)
{
    sample.Clear();
    bool bUsingBestSolution = false;
    if(bBestSolution && problem.m_pBestSolution != NULL && problem.m_pBestSolution->m_Sample.m_vCommands.size() != 0){
        bUsingBestSolution = true;
        sample.m_vCommands = problem.m_pBestSolution->m_Sample.m_vCommands;
    }else{
        SampleAcceleration(sample.m_vCommands, problem);
    }

    VehicleState vState;
    if(sample.m_vCommands.size() == 0){
        vState = problem.m_StartState;
    }else {
        vState = problem.m_pFunctor->ApplyVelocities( problem.m_StartState, sample, nIndex, bUsingBestSolution);
    }
    //transform the result back
    Eigen::Vector6d dRes = _Transform3dGoalPose(vState,problem);
    return dRes;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::_GetAccelerationProfile(LocalProblem& problem) const
{
    double totalDist = problem.m_BoundaryProblem.m_dDistance;
    double currentDist = 0;
    double totalTime = 0;

    //must have at least two nodes
    assert(problem.m_vVelProfile.size()>1);

    //prepare the accel profile
    problem.m_vAccelProfile.clear();
    problem.m_vAccelProfile.reserve(problem.m_vVelProfile.size());

    //first calcualte the segment time
    for(size_t ii = 1; ii < problem.m_vVelProfile.size(); ii++){
        //calculate the distance in this segment
        double segDist = (problem.m_vVelProfile[ii].m_dDistanceRatio - problem.m_vVelProfile[ii-1].m_dDistanceRatio)*totalDist;
        double segTime = segDist / (problem.m_vVelProfile[ii-1].m_dVel + 0.5 * (problem.m_vVelProfile[ii].m_dVel - problem.m_vVelProfile[ii-1].m_dVel));
        totalTime += segTime;
        currentDist += segDist;

        //push back the accel profile
        double accel = (problem.m_vVelProfile[ii].m_dVel - problem.m_vVelProfile[ii-1].m_dVel)/segTime;
        problem.m_vAccelProfile.push_back(AccelerationProfileNode(totalTime,accel,currentDist,problem.m_vVelProfile[ii-1].m_dVel,problem.m_vVelProfile[ii].m_dVel));
    }
    problem.m_dSegmentTime = totalTime;
    //problem.m_dMaxSegmentTime = DBL_MAX;
}

///////////////////////////////////////////////////////////////////////
bool LocalPlanner::InitializeLocalProblem(LocalProblem& problem,
                                          const double dStartTime,
                                          const VelocityProfile* pVelProfile /* = NULL*/,
                                          LocalProblemCostMode eCostMode /*= eCostPoint */)
{
    problem.Reset();
    problem.m_nPlanId = m_nPlanCounter++;

    //if there are previous commands, apply them so we may make a more educated guess
    MotionSample delaySample;
    double totalDelay = problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
    if(totalDelay > 0 && problem.m_pFunctor->GetPreviousCommand().size() != 0){
        CommandList::iterator it  = problem.m_pFunctor->GetPreviousCommand().begin();
        while(totalDelay > 0 && it != problem.m_pFunctor->GetPreviousCommand().end()){
            delaySample.m_vCommands.insert(delaySample.m_vCommands.begin(),(*it));
            //delaySample.m_vCommands.push_back((*it)  );
            totalDelay -= (*it).m_dT;
            ++it;
        }
        problem.m_pFunctor->ResetPreviousCommands();
        problem.m_pFunctor->SetNoDelay(true);
        problem.m_pFunctor->ApplyVelocities(problem.m_StartState,delaySample,0,true);
        //and now set the starting state to this new value
        problem.m_StartState = delaySample.m_vStates.back();
    }

    //regardless of the delay, for local planning we always want to proceed with no delay and with no previous commands
    //as the previous section should take care of that
    problem.m_pFunctor->ResetPreviousCommands();
    problem.m_pFunctor->SetNoDelay(true);

    Sophus::SE3d dTranslation(Sophus::SO3d(),-problem.m_StartState.m_dTwv.translation());

    //first translate to base everything around dStartPose
    Sophus::SE3d dFixedStart = problem.m_StartState.m_dTwv;
    Sophus::SE3d dFixedGoal = problem.m_GoalState.m_dTwv;

    //now rotate everything to a frame inbetween the two
    Eigen::Quaternion<double> dStartQuat = problem.m_StartState.m_dTwv.so3().unit_quaternion();
    Eigen::Quaternion<double> dEndQuat = problem.m_GoalState.m_dTwv.so3().unit_quaternion();
    //find the halfway rotation
    Eigen::Quaternion<double> dMidQuat = dStartQuat.slerp(0.5,dEndQuat);
    //dMidQuat.setIdentity();

    //now rotate both waypoints into this intermediate frame
    Sophus::SE3d dRotation(Sophus::SO3d(dMidQuat.toRotationMatrix().transpose()),Eigen::Vector3d::Zero()); //we want the inverse rotation here
    static bool& bFlatten2Dcurves = CVarUtils::CreateGetCVar("debug.flatten2Dcurves",false,"");
    if(bFlatten2Dcurves){
        dRotation.so3() = Sophus::SO3d();
    }

    problem.m_dT3d = (dRotation*dTranslation);
    problem.m_dT3dInv = problem.m_dT3d.inverse();

    //now rotate both points;
    dFixedStart = problem.m_dT3d * dFixedStart;
    dFixedGoal = problem.m_dT3d * dFixedGoal;

    VehicleState dStartStateFixed = problem.m_StartState;
    VehicleState dGoalStateFixed = problem.m_GoalState;
    dStartStateFixed.m_dTwv = dFixedStart;
    dGoalStateFixed.m_dTwv = dFixedGoal;
    //and now we can project to 2d and get our 2d planner points
    Eigen::Vector6d dStartPose2D = dStartStateFixed.ToPose();
    Eigen::Vector6d dGoalPose2D = dGoalStateFixed.ToPose();

    //make sure we don't have start and end velocities of zero
    if(dStartPose2D[5] == 0 && dGoalPose2D[5] == 0){
        return false;
    }

    problem.m_dStartTime = dStartTime;

    //setup the boundary problem
    problem.m_dStartPose = dStartPose2D;
    problem.m_dGoalPose = dGoalPose2D;
    problem.m_pBoundarySovler = &m_BoundarySolver;

    problem.m_dTinv = rpg::TInv( rpg::Cart2T( problem.m_dStartPose[0], problem.m_dStartPose[1], problem.m_dStartPose[3] ));
    Eigen::Vector6d dTemp = _TransformGoalPose(dStartPose2D,problem);
    problem.m_BoundaryProblem.m_dStartPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    dTemp  = _TransformGoalPose(dGoalPose2D,problem);
    problem.m_BoundaryProblem.m_dGoalPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    problem.m_BoundaryProblem.m_nDiscretization = 50;

    if(pVelProfile == NULL){
        problem.m_vVelProfile.clear();
        problem.m_vVelProfile.reserve(2);
        problem.m_vVelProfile.push_back(VelocityProfileNode(0,problem.m_dStartPose[5]));
        problem.m_vVelProfile.push_back(VelocityProfileNode(1,problem.m_dGoalPose[5]));
    }else{
        problem.m_vVelProfile = *pVelProfile;
    }



    //this puts the result in m_dP
    problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);
    _GetAccelerationProfile(problem);

    //initialize the max time
    problem.m_dMaxSegmentTime = problem.m_dSegmentTime*5;

    //reset optimization parameters
    problem.m_CurrentSolution.m_dNorm = DBL_MAX;
    problem.m_bInLocalMinimum = false;
    problem.m_CurrentSolution.m_dOptParams.head(3) = problem.m_BoundaryProblem.m_dGoalPose.head(3);
    if(OPT_DIM > OPT_AGGR_DIM){
        problem.m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM] = problem.m_BoundaryProblem.m_dAggressiveness;
    }
    problem.m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM] = 0.0; // this is the percentage of the calculated acceleration
    problem.m_dInitOptParams = problem.m_CurrentSolution.m_dOptParams;
    problem.m_pBestSolution = &problem.m_CurrentSolution;

    //the transformed final position
    problem.m_dTransformedGoal = _TransformGoalPose(problem.m_dGoalPose, problem);

    //restore the original start state for simulation purposes
    //problem.m_StartState = originalStartState;
    problem.m_eCostMode = eCostMode;

    return true;
}


///////////////////////////////////////////////////////////////////////
bool LocalPlanner::Iterate(LocalProblem &problem )
{
    try
    {
        if(problem.m_CurrentSolution.m_dNorm < g_dSuccessNorm) {
            DLOG(INFO) << problem.m_nPlanId << ":Succeeded to plan. Norm = " << problem.m_CurrentSolution.m_dNorm;
            return true;
        }

        //get the current state and norm for the first iteration
        if(problem.m_lSolutions.size() == 0 ) {
            //here we have to re-evaluate the segment time of the trajectory

            SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem,0,false);
            //problem.m_dDistanceDelta = problem.m_pCurrentMotionSample->GetDistance() - problem.m_BoundaryProblem.m_dDistance;
            if(problem.m_bInertialControlActive){
                CalculateTorqueCoefficients(problem,&problem.m_CurrentSolution.m_Sample);
                SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem,0,false);
            }

            //calculate the distance
            double dMinLookahead;
            Eigen::VectorXd error = _CalculateSampleError(problem,dMinLookahead);
            //DLOG(INFO) << problem.m_nPlanId << ":Initial error is " << error.transpose();
            problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,error);
            if(std::isfinite(problem.m_CurrentSolution.m_dNorm) == false){
                DLOG(INFO) << problem.m_nPlanId << ":Initial norm is not finite. Exiting optimization";
                return false;
            }
            problem.m_lSolutions.push_back(problem.m_CurrentSolution);
            problem.m_pBestSolution = &problem.m_lSolutions.back();
        }

        if( m_dEps > 5 || problem.m_bInLocalMinimum == true) {
            //DLOG(INFO) << "Failed to plan. Norm = " << problem.m_dCurrentNorm;
            return true;
        }

        _IterateGaussNewton(problem);
        if(problem.m_bInertialControlActive){
            CalculateTorqueCoefficients(problem,&problem.m_CurrentSolution.m_Sample);
        }
        return false;
    }catch(...){
        return false;
    }
}

void LocalPlanner::CalculateTorqueCoefficients(LocalProblem& problem,MotionSample* pSample)
{
    //find the last air section and record its duration
    double dAirTime = 0;
    int nStartIndex = -1;
    double dStartTime = 0;

    double dLongestAirTime = 0, dLongestStartTime = 0;
    int nLongestStartIndex = -1;

    //find the longest airborne segment in the trajectory
    for(int ii = std::min(pSample->m_vStates.size()-1,
                          pSample->m_vCommands.size()-1) ;
            ii>=0 ; --ii){
        if(pSample->m_vStates[ii].IsAirborne()){
            dAirTime += pSample->m_vCommands[ii].m_dT;
            nStartIndex = ii;
            dStartTime = pSample->m_vStates[ii].m_dTime;
        }else{
            if(dAirTime != 0){
                if(dAirTime > dLongestAirTime){
                    dLongestAirTime = dAirTime;
                    dLongestStartTime = dStartTime;
                    nLongestStartIndex = nStartIndex;

                    dAirTime = 0;
                    nStartIndex = -1;
                    dStartTime = 0;
                }
            }
        }
    }

    if(nLongestStartIndex != -1){
        dAirTime = dLongestAirTime;
        dStartTime = dLongestStartTime;
        nStartIndex = nLongestStartIndex;
    }

    //now calcualte the coefficients for this maneuver based on the angle error
    if(dAirTime > 0.05){
        //find the goal state based on the cost type
        Sophus::SO3d dGoalState;
        if(problem.m_eCostMode == eCostPoint){
            dGoalState = problem.m_GoalState.m_dTwv.so3();
        }else{
            dGoalState = problem.m_GoalState.m_dTwv.so3();

            bool bCurrentAirborne = problem.m_Trajectory.m_vStates[0].IsAirborne();
            for(size_t ii = 1 ; ii < problem.m_Trajectory.m_vStates.size() ; ii++){
                bool bTemp = problem.m_Trajectory.m_vStates[ii].IsAirborne();
                if(bTemp == false && bCurrentAirborne == true){
                    dGoalState = problem.m_Trajectory.m_vStates[ii].m_dTwv.so3();
                    break;
                }
                bCurrentAirborne = bTemp;
            }
        }
        //calculate the change in angles necessary
        const Sophus::SO3d Rw_dest = problem.m_GoalState.m_dTwv.so3();
        const Sophus::SO3d Rw_init = pSample->m_vStates[nStartIndex].m_dTwv.so3();

        //const Sophus::SO3d Rinit_w = Rw_init.inverse();
        //const Sophus::SO3d Rinit_dest = Rinit_w * Rw_dest;
        //angle in body frame
        const Eigen::Vector3d angles(0,
                                     rpg::AngleWrap(rpg::R2Cart(Rw_dest.matrix())[1]-rpg::R2Cart(Rw_init.matrix())[1]),// - (problem.m_eCostMode == eCostPoint ? M_PI*2 : 0),
                                     0);// = Rinit_dest.log();

        const Eigen::Vector3d dInertia = problem.m_pFunctor->GetCarModel()->GetVehicleInertiaTensor(0);
        //Eigen::Vector3d currentV = pSample->m_vStates[nStartIndex].m_dV;
        const Sophus::SO3d Rwv = pSample->m_vStates[nStartIndex].m_dTwv.so3();
        //angular velocities in body frame
        //create the inertia tensor
//        Eigen::Matrix3d omega_w;
//        omega_w << 0, -pSample->m_vStates[nStartIndex].m_dW[2], pSample->m_vStates[nStartIndex].m_dW[1],
//                   pSample->m_vStates[nStartIndex].m_dW[2],0,-pSample->m_vStates[nStartIndex].m_dW[0],
//                   -pSample->m_vStates[nStartIndex].m_dW[1],pSample->m_vStates[nStartIndex].m_dW[0],0;
        //DLOG(INFO) << "omega " << omega_w;

        //Sophus::SO3d dRwv = Sophus::SO3d::hat(pSample->m_vStates[nStartIndex].m_dW);
        //const Eigen::Vector3d omega_v =  Sophus::SO3d::vee((Rwv.inverse()*dRwv).matrix());
        const Eigen::Vector3d omega_v = Rwv.inverse() * pSample->m_vStates[nStartIndex].m_dW;

        //calculate the coefficients
        Eigen::Matrix4d A;
        Eigen::Vector4d B;

        A << 1, 0              ,0                     ,0,
                1, dAirTime, powi(dAirTime,2), powi(dAirTime,3),
                dAirTime, powi(dAirTime,2)/2,powi(dAirTime,3)/3,powi(dAirTime,4)/4,
                powi(dAirTime,2)/2,  powi(dAirTime,3)/6,powi(dAirTime,4)/12,powi(dAirTime,5)/20;

            Eigen::Matrix4d Ainertia = A / dInertia(1);
            B << problem.m_dStartTorques(1),0, -omega_v(1), angles(1) - omega_v(1)*dAirTime;
            problem.m_dCoefs = Ainertia.inverse() * B;
            problem.m_dTorqueStartTime = dStartTime;
            problem.m_dTorqueDuration = dAirTime;
    }else{
        problem.m_dTorqueStartTime = -1;
    }
}

void LocalPlanner::StressTest(LocalProblem &problem)
{
//    std::vector<std::shared_ptr<boost::thread> > threads;
//    for(int jj = 0; jj < 1000 ; jj++){
//        Eigen::Vector6d pHypeStates[DAMPING_STEPS];
//        Eigen::VectorXd pHypeErrors[DAMPING_STEPS];
//        std::vector<std::shared_ptr<LocalProblem> > vCubicProblems;
//        std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
//        vCubicProblems.resize(DAMPING_STEPS);
//        vFunctors.resize(DAMPING_STEPS);

////        vCubicProblems[0] = std::make_shared<LocalProblem>(problem);
////        vFunctors[0] = std::make_shared<ApplyCommandsThreadFunctor>(this,*vCubicProblems[0],0,pHypeStates[0],m_DampingMotionSamples[0],true);
////        vFunctors[0]->operator()();

//        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
//            vCubicProblems[ii] = std::make_shared<LocalProblem>(problem);
//            vCubicProblems[ii]->m_pBoundarySovler->Solve(&vCubicProblems[ii]->m_BoundaryProblem);
//            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(this,*vCubicProblems[ii],ii,pHypeStates[ii],pHypeErrors[ii],m_MotionSamples[ii],false);
//            threads.push_back(std::make_shared<boost::thread>(*vFunctors[ii].get()));
//            //vFunctors[0]->operator()();
//            //m_ThreadPool.schedule(*vFunctors[ii]);
//        }

//        for(size_t ii = 0 ; ii < threads.size() ; ii++){
//            threads[ii]->join();
//        }
//        threads.clear();
//        //m_ThreadPool.wait();
//    }
}


///////////////////////////////////////////////////////////////////////
bool LocalPlanner::_IterateGaussNewton( LocalProblem& problem )
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd dDeltaP;
    //        DLOG(INFO) << "Entered gauss-newton search with e = " << m_dCurrentEps;
    double bestDamping;
    unsigned int nBestDampingIdx = 0;

    Eigen::VectorXd dWvec = _GetWeightVector(problem);

    Eigen::MatrixXd J = Eigen::MatrixXd(dWvec.rows(),OPT_DIM);
    //create an appropriate weighting matrix
    Eigen::MatrixXd dW = dWvec.asDiagonal();

    //double dMinLookahead;
    Eigen::VectorXd error;// = _CalculateSampleError(problem,dMinLookahead);

    //DLOG(INFO) << problem.m_nPlanId << ":Iteration error is" << error.transpose();
    LocalProblemSolution coordinateDescent;
    if(_CalculateJacobian(problem,error,coordinateDescent,J) == false){
        return false;
    }

    problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,error);
    if(g_bVerbose){
        DLOG(INFO) << "Calculated jacobian with base norm: " << problem.m_CurrentSolution.m_dNorm;
    }

    //if any of the columns are zero, reguralize
    bool bZeroCols = false;
    for(int ii = 0 ; ii < J.cols() ; ii++){
        if( J.col(ii).norm() == 0){
            bZeroCols = true;
            break;
        }
    }


    //solve for the dDeltaP
    dDeltaP = J.transpose()*dW*error;
    Eigen::MatrixXd JtJ = J.transpose()*dW*J;
    //Do thikonov reguralization if there is a null column
    if(bZeroCols) {
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
    }
    (JtJ).llt().solveInPlace(dDeltaP);

    //this is a hack for now, but it should take care of large deltas
    if(dDeltaP.norm() > 100 ){
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
        dDeltaP = J.transpose()*dW*error;
        (JtJ).llt().solveInPlace(dDeltaP);
    }

    if(g_bVerbose){
        DLOG(INFO) << "Gauss newton delta: [" << dDeltaP.transpose().format(CleanFmt) << "]";
    }


    if(std::isfinite(dDeltaP.norm()) == false){
        DLOG(ERROR) << problem.m_nPlanId << ":Deltas are NAN. Dump => J:" << J.format(CleanFmt) << std::endl << "b:" << error.transpose().format(CleanFmt) << std::endl;
        problem.m_eError = eDeltaNan;
        return false;
    }

    //if no damping is required, just pick the result of the gauss newton
    if(g_bDisableDamping == true){
//        problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM) += dDeltaP.head(OPT_DIM);
//        problem.m_BoundaryProblem.m_dGoalPose.head(3) = problem.m_CurrentSolution.m_dOptParams.head(3);
//        problem.m_BoundaryProblem.m_dAggressiveness = problem.m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM];
        problem.UpdateOptParams(problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM) + dDeltaP);
        problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);
        SimulateTrajectory( problem.m_CurrentSolution.m_Sample,problem);
        problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,_CalculateSampleError(problem,problem.m_CurrentSolution.m_dMinTrajectoryTime));
        problem.m_lSolutions.push_back(problem.m_CurrentSolution);
        problem.m_pBestSolution = &problem.m_lSolutions.back();
        DLOG(INFO) << "Iteration with no damping finished with norm " << problem.m_CurrentSolution.m_dNorm << " and opts "<< problem.m_CurrentSolution.m_dOptParams.transpose().format(CleanFmt);
        return true;
    }

    //initialize the parameters here, in case non of the
    //dampings are any good
    double damping = 0;
    //damp the gauss newton response
    //DLOG(INFO) << "Gauss newton delta is: [" << dDeltaP.transpose() << "] - Damping with lambda";

    Eigen::Vector6d pDampingStates[DAMPING_STEPS];
    Eigen::VectorXd pDampingErrors[DAMPING_STEPS];
    //Eigen::VectorOpt pHypePs[DAMPING_STEPS];
    std::vector<std::shared_ptr<LocalProblem> > vCubicProblems;
    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
    vCubicProblems.resize(DAMPING_STEPS);
    vFunctors.resize(DAMPING_STEPS);
    double dampings[DAMPING_STEPS];
    damping = 1.0;


    LocalProblemSolution dampedSolution;
    dampedSolution.m_dNorm = DBL_MAX;
    if(std::isfinite(dDeltaP[0]) ){
        //create the damped problems and run the thread queue to sample them
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaP *damping;
            vCubicProblems[ii] = std::make_shared<LocalProblem>(problem);
            vCubicProblems[ii]->UpdateOptParams(vCubicProblems[ii]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                         *vCubicProblems[ii],
                                                                         ii,
                                                                         pDampingStates[ii],
                                                                         pDampingErrors[ii],
                                                                         vCubicProblems[ii]->m_CurrentSolution.m_Sample,
                                                                         true);
            m_ThreadPool.schedule(*vFunctors[ii]);
            damping/= DAMPING_DIVISOR;
        }
        m_ThreadPool.wait();


        if(g_bVerbose){
            std::cout << "Damping norms are: [";
        }
        //pick out the best damping by comparing the norms
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            double norm = 0;
            dout_cond("Final state is " << pDampingStates[ii].transpose().format(CleanFmt),g_bVerbose);
            norm = _CalculateErrorNorm(*vCubicProblems[ii],pDampingErrors[ii]);
            if(g_bVerbose){
                std::cout << " " << norm;
            }
            if(norm < dampedSolution.m_dNorm ) {
                dampedSolution = vCubicProblems[ii]->m_CurrentSolution;
                dampedSolution.m_dNorm = norm;
                bestDamping = dampings[ii];
                nBestDampingIdx = ii;
            }
        }

        if(g_bVerbose){
            std::cout << "]" << std::endl;
        }

    }else{
        Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        DLOG(ERROR) << problem.m_nPlanId << ":Deltas are NAN. Dump => J:" << J.format(CleanFmt) << std::endl;
        problem.m_eError = eDeltaNan;
        return false;
    }

    LocalProblemSolution newSolution;

    if(coordinateDescent.m_dNorm > problem.m_CurrentSolution.m_dNorm && g_bMonotonicCost){
        problem.m_bInLocalMinimum = true;
        DLOG(INFO) << problem.m_nPlanId << ":In local minimum with Norm = " << problem.m_CurrentSolution.m_dNorm;
    }else if( dampedSolution.m_dNorm > problem.m_CurrentSolution.m_dNorm && g_bMonotonicCost) {
        //newSolution = coordinateDescent;
        problem.m_bInLocalMinimum = true;
        DLOG(INFO) << problem.m_nPlanId << ":Accepted coordinate descent with norm = " << problem.m_CurrentSolution.m_dNorm;
    }else{
        newSolution = dampedSolution;
        DLOG(INFO) <<  problem.m_nPlanId << ":New norm from damped gauss newton = " << newSolution.m_dNorm << " with damping = " << bestDamping << " best damped traj error is " << pDampingErrors[nBestDampingIdx].transpose().format(CleanFmt);
    }


    //update the problem params
    //problem.m_CurrentSolution.m_dOptParams = newSolution.m_dOptParams;


    //update the best solution if necessary
    if(problem.m_bInLocalMinimum == false){
        if(newSolution.m_dNorm < problem.m_CurrentSolution.m_dNorm){
            //add this solution to the list
            problem.m_lSolutions.push_back(newSolution);
            problem.m_pBestSolution = &problem.m_lSolutions.back();
        }
        problem.m_CurrentSolution = newSolution;
    }
    //problem.m_CurrentSolution.m_Sample = newSolution.m_Sample;

    problem.UpdateOptParams(problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM));
    problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);

//    if(problem.m_pCurrentMotionSample == NULL) {
//        assert(false);
//    }
    problem.m_eError = eSuccess;
    return true;
}


