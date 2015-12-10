#include <CarPlanner/CarRegressor.h>

static bool g_bUseCentralDifferences = false;
static bool g_bCurvatureDependentSegmentation = false;
static int g_nSegmentLength = 80;

struct ApplyParametersThreadFunctor {
    ApplyParametersThreadFunctor(CarRegressor* pRegressor,
                                 ApplyVelocitesFunctor5d &f,
                                 MotionSample &plan,
                                 const std::vector<RegressionParameter>& params,
                                 const int index,
                                 Eigen::Vector7d& errorOut,
                                 int nStartIndex,
                                 int nEndIndex,
                                 CommandList* pPreviousCommands,
                                 std::vector<VehicleState>* pStatesOut = NULL) :
        m_pRegressor(pRegressor),
        m_f(f),
        m_plan(plan),
        m_params(params),
        m_index(index),
        m_errorOut(errorOut),
        m_nStartIndex(nStartIndex),
        m_nEndIndex(nEndIndex),
        m_pPreviousCommands(pPreviousCommands),
        m_pStatesOut(pStatesOut)
    {}

    void operator()()
    {
        //make sure we don't go over the number of worlds we have
        if( m_index >= m_f.GetCarModel()->GetWorldCount()){
            assert(false);
        }
        m_pRegressor->ApplyParameters(m_f,m_plan,m_params,m_index,m_errorOut,m_nStartIndex,
                                      m_nEndIndex,m_pStatesOut,m_pPreviousCommands);
    }

    CarRegressor* m_pRegressor;
    ApplyVelocitesFunctor5d &m_f;
    MotionSample& m_plan;
    const std::vector<RegressionParameter>& m_params;
    const int m_index;
    Eigen::Vector7d& m_errorOut;
    int m_nStartIndex;
    int m_nEndIndex;
    CommandList* m_pPreviousCommands;
    std::vector<VehicleState>* m_pStatesOut;   
};

///////////////////////////////////////////////////////////////////////
CarRegressor::CarRegressor() :
  m_ThreadPool(REGRESSOR_NUM_THREADS)
{ }

///////////////////////////////////////////////////////////////////////
void CarRegressor::Init(double dEpsilon, std::fstream *pLogFile)
{
  //TODO(crh): Move to MochaGui.
  /*
  pangolin::Var<bool>::Attach("debug.UseCentralDifferences",g_bUseCentralDifferences);
  pangolin::Var<bool>::Attach("learning.CurvatureDependentSegmentation",g_bCurvatureDependentSegmentation);
  pangolin::Var<int>::Attach("learning.SegmentLength",g_nSegmentLength);
  */

  m_dCurrentNorm = NORM_NOT_INITIALIZED;
  m_bFailed = false;

  //initialize the weight matrix
  m_dW = Eigen::MatrixXd::Identity(7,7);
  //give the position terms a higher weight
  //    m_dW(0,0) = m_dW(1,1) = m_dW(2,2) = XY_MULTIPLIER;
  //    m_dW(3,3) = m_dW(4,4) = m_dW(5,5) = THETA_MULTIPLIER;
  //    m_dW(6,6) = VEL_MULTIPLIER;

  m_dEpsilon = dEpsilon;
  m_pLogFile = pLogFile;
}

///////////////////////////////////////////////////////////////////////
void CarRegressor::_RefreshIndices(MotionSample &sample,ApplyVelocitesFunctor5d& f)
{
    m_nSegmentLength = g_nSegmentLength;

    //find out the minimum starting index so that we can do the maximum control
    //delay if needed
    double totalDelay = MAX_CONTROL_DELAY;
    for(m_nStartIndex = 0 ; m_nStartIndex < (int)sample.m_vCommands.size() && totalDelay >= 0 ; m_nStartIndex++){
        totalDelay -= sample.m_vCommands[m_nStartIndex].m_dT;
    }

    m_vSegmentIndices.clear();
    //designate the semgnet sections
    for(size_t startIndex = 0 ;  startIndex < sample.m_vCommands.size() ; ){
         //this is to ensure we never start before m_nStartIndex
        startIndex = std::max((int)startIndex,m_nStartIndex);
        //this is to ensure the minimum segment length is defined by m_nSegmentLength
        int endIndex = startIndex + m_nSegmentLength;
        //search the commands array to find a location which has close to 0 steering, to end this segment
        if(g_bCurvatureDependentSegmentation){
            for(size_t jj = endIndex; jj < sample.m_vCommands.size() ; jj++){
                //if the turn radius is above 10m, then this is a good place to end this segment
                endIndex = jj;
                if(fabs(sample.m_vCommands[jj].m_dCurvature) < 0.5 ){
                    //then find out when the actual transition occurs, as we need
                    //to factor in control delay (Note: this is approximate)
                    double dDelay = 0;
                    for(size_t kk = jj ;
                        kk < sample.m_vCommands.size() &&
                        dDelay < f.GetCarModel()->GetParameters(0)[CarParameters::ControlDelay]  ;
                        kk++){
                        endIndex = jj;
                    }
                    break;
                }
            }
        }
        //ensure we never exceed the end of the command array
        endIndex = std::min(endIndex,(int)sample.m_vCommands.size());
        //dout("Indices for segment " << m_vSegmentIndices.size() << " are " << startIndex << " to " << endIndex);
        m_vSegmentIndices.push_back(std::pair<int,int>(startIndex, endIndex));
        startIndex = endIndex;
    }
}

///////////////////////////////////////////////////////////////////////
void CarRegressor::Regress(ApplyVelocitesFunctor5d& f,
                           MotionSample &sample,
                           const std::vector<RegressionParameter>& params,
                           std::vector<RegressionParameter>& newParams)
{
    _RefreshIndices(sample,f);
    //m_vSegmentIndices.push_back(std::pair<int,int>(0, plan.m_vCommands.size()-1));


    std::vector<RegressionParameter> origParams = params;
    std::vector<RegressionParameter> updatedParams;

    newParams = params;
    m_dCurrentNorm = CalculateParamNorms(f,sample,params);

    DLOG(INFO) << "Staring regression on " << sample.m_vStates.size() << " data points starting with params=" << params;
    m_bFailed = false;
    double dLastNorm = 0;
    int maxIter = 500;
    for(int ii = 0 ; ii < maxIter && m_bFailed == false ; ii++){
        dLastNorm = m_dCurrentNorm;
        _OptimizeParametersGN(f,sample,newParams,updatedParams,m_dCurrentNorm);
        //write the new parameters back
        DLOG(INFO) << "parameter update: " << updatedParams << " norm:" << m_dCurrentNorm;
        newParams = updatedParams;

        //if the reduction is too little
        if(fabs(dLastNorm - m_dCurrentNorm)/dLastNorm < 0.000001){
            DLOG(INFO) << "Norm changed too little. Exiting optimization.";
            break;
        }
    }

    DLOG(INFO) << "orig params: " << origParams << ". new params " << newParams << " norm:" << m_dCurrentNorm;
}

///////////////////////////////////////////////////////////////////////
double CarRegressor::CalculateParamNorms(ApplyVelocitesFunctor5d simFunctor,
                                         MotionSample& sample,
                                         const std::vector<RegressionParameter>& params,
                                         std::vector<MotionSample>* pSamples /* = NULL */,
                                         std::vector<int>* pSampleIndices/* = NULL */)
{
    _RefreshIndices(sample,simFunctor);

    double dTotalNorm = 0;
    Eigen::Vector7dAlignedVec dErrors;
    //reserve ample space
    dErrors.resize(m_vSegmentIndices.size());
    //reserve space for the motion samples if required
    if(pSamples != NULL){
        pSamples->resize(dErrors.size());
    }
    if(pSampleIndices != NULL){
        pSampleIndices->resize(dErrors.size());
    }

    std::vector<CommandList> vPreviousCommands;
    vPreviousCommands.resize(dErrors.size());
    int counter = 0, worldCounter = 0;
    for(size_t ii = 0; ii < m_vSegmentIndices.size() ; ii++){
        int startIndex = m_vSegmentIndices[ii].first;
        int endIndex = m_vSegmentIndices[ii].second;

        //queue the previous commands (based on the maximum control delay time)
        vPreviousCommands[counter] = sample.GetDelayedCommandList(MAX_CONTROL_DELAY,startIndex);

        //if we have no more worlds, wait until all worlds have finished
        if(worldCounter >= simFunctor.GetCarModel()->GetWorldCount()) {
          while(m_ThreadPool.busy_threads() > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
            //augment the counter
            worldCounter  = 0;
        }

        //calculate the error for this stretch
        ApplyParametersThreadFunctor functor(this,
                                             simFunctor,
                                             sample,
                                             params,
                                             worldCounter,
                                             dErrors[counter],
                                             startIndex,endIndex,
                                             &vPreviousCommands[counter],
                                             pSamples == NULL? NULL : &pSamples->at(counter).m_vStates);

        if(pSampleIndices != NULL){
            pSampleIndices->at(counter) = startIndex;
        }

        //schedule this calculation
        m_ThreadPool.enqueue(functor);

        counter++;
        worldCounter++;
    }

    while(m_ThreadPool.busy_threads() > 0){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    //go through all errors and add them up (include the weight factor)
    for(int ii = 0, s = dErrors.size(); ii < s ; ii++){
        //calculate the weight based on the length of the segment
        dTotalNorm += dErrors[ii].norm();
    }

    return dTotalNorm;
}

///////////////////////////////////////////////////////////////////////
void CarRegressor::_OptimizeParametersGN(ApplyVelocitesFunctor5d& f,
                                         MotionSample& plan,
                                         const std::vector<RegressionParameter>& dParams,
                                         std::vector<RegressionParameter>& dParamsOut,
                                         double& dNewNorm)
{
    //        dout("Entered gauss-newton search with e = " << m_dCurrentEps);
    Eigen::IOFormat CleanFmt(5, 0, ", ", "\n", "[", "]");
    double dBestNorm = m_dCurrentNorm,dBestDamping;
    dParamsOut = dParams;
    int nBestDimension;
    Eigen::VectorXd dDeltaParams;
    std::vector<RegressionParameter> dBestParams, dHypeParams;
    Eigen::MatrixXd JtJ = Eigen::MatrixXd::Zero(dParams.size(),dParams.size());
    Eigen::VectorXd Jb = Eigen::MatrixXd::Zero(dParams.size(),1);
    CalculateJacobian(f,plan,dParams,dBestParams,dBestNorm,nBestDimension,JtJ,Jb);

    //if the prior has not been initialized, simply set it to identity to start
    if(m_dPrior.rows() != JtJ.rows()){
        m_dPrior = Eigen::MatrixXd(JtJ.rows(),JtJ.cols());
        m_dPrior.setIdentity();
    }

    //add the prior to the optimization here
    //JtJ += m_dPrior;

    dDeltaParams = Jb;
    JtJ.llt().solveInPlace(dDeltaParams);

    //afterwards, add the current information to the prior
    m_dPrior = m_dPrior+JtJ*0.1;

    double dHypeNorm = DBL_MAX;
    double damping = 0;
    if(std::isfinite(dDeltaParams[0]) ){
        //damp the gauss newton response
        DLOG(INFO) << "Gauss newton delta is : " << dDeltaParams.transpose().format(CleanFmt) << " norm: " << dBestNorm;
        double dHypeNorms[DAMPING_STEPS];
        std::vector<RegressionParameter> pHypeParams[DAMPING_STEPS];
        double dampings[DAMPING_STEPS];
        damping = 1.0;
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaParams *damping;
            pHypeParams[ii] = dParams;
            //add the delta to the parameters
            for(int jj = 0 ; jj < delta.rows() ; jj++){
                pHypeParams[ii][jj].m_dVal -= delta[jj];
            }
            //pHypeParams[ii] = dParams - delta;
            dHypeNorms[ii] = CalculateParamNorms(f,plan,pHypeParams[ii]);
            //ApplyParametersThreadFunctor functor(this,f,plan,pHypeParams[ii],ii,dHypeNorms[ii],0,(int)plan.m_vDts.size());
            //m_ThreadPool.schedule(functor);
            damping/= DAMPING_DIVISOR;
        }

        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            double norm = dHypeNorms[ii];
            if(norm < dHypeNorm ) {
                dHypeParams = pHypeParams[ii];
                dHypeNorm = norm;
                dBestDamping = dampings[ii];
            }
        }
    }else{
        DLOG(INFO) << "NaN returned";
        m_bFailed = true;
    }

    if( dHypeNorm > dBestNorm ) {

        if(RegressionParameter::AreEqual(dParams,dBestParams)) {
            //m_bFailed = true;
        }else{
            //steepest descent
            m_eCurrentTask = eGaussNewton;
            DLOG(INFO) << "coord descent params: " << dBestParams << " norm: " << dBestNorm;
        }
        dParamsOut = dBestParams;
        dNewNorm = dBestNorm;
    }else {
        //accept the damped gauss newton
        dParamsOut = dHypeParams;
        dNewNorm = dHypeNorm;
        m_eCurrentTask = eGaussNewton;
        DLOG(INFO) << "GN params: " << dHypeParams << " damping: " << dBestDamping <<  " norm: " << dHypeNorm;
    }
}

///////////////////////////////////////////////////////////////////////
void CarRegressor::ApplyParameters(ApplyVelocitesFunctor5d f,
                                   MotionSample& plan,
                                   const std::vector<RegressionParameter>& params,
                                   const int index,
                                   Eigen::Vector7d& errorOut,
                                   int nStartIndex,
                                   int nEndIndex,
                                   std::vector<VehicleState>* pStatesOut  /*= NULL*/,
                                   CommandList *pPreviousCommands /*= NULL*/)
{
    //set the new parameters on the car
    f.GetCarModel()->UpdateParameters(params,index);

    std::vector<VehicleState> vStates;
    std::vector<VehicleState>& vStatesOut = pStatesOut == NULL ? vStates : *pStatesOut;

    //apply the velocities WITHOUT COMPENSATION and with no torque calculation, this is so that we apply
    //the same torques/accels to the regressor vehicle as the real one
    f.ApplyVelocities(plan.m_vStates[nStartIndex],
                      plan.m_vCommands,
                      vStatesOut,
                      nStartIndex,
                      nEndIndex,
                      index,
                      true,
                      pPreviousCommands);

    errorOut.setZero();
    for(size_t ii = 0 ; ii < vStatesOut.size() ; ii+= 10){
      Eigen::Vector6d error6d;
      //dout("Error out " << error6d.transpose() << std::endl);
      errorOut.head(6) += error6d;
      errorOut[6] += (vStatesOut[ii].m_dV.norm() - plan.m_vStates[nStartIndex+ii].m_dV.norm());
    }
}

///////////////////////////////////////////////////////////////////////
void CarRegressor::CalculateJacobian(ApplyVelocitesFunctor5d functor,
                                     MotionSample& plan,
                                     const std::vector<RegressionParameter>& params,
                                     std::vector<RegressionParameter>& vBestParams,
                                     double& dBestNorm,
                                     int& nBestDimension,
                                     Eigen::MatrixXd& JtJ,
                                     Eigen::VectorXd& Jb)
{
    Eigen::IOFormat CleanFmt(5, 0, ", ", "\n", "[", "]");
    //int nControlDelayParamIndex = -1;
    Eigen::Vector7d dBaseError;
    //create a copy of the parameter vector for perturvation
    std::vector<std::shared_ptr<ApplyParametersThreadFunctor> > vFunctors;
    std::vector<std::vector<RegressionParameter> > vParams;
    Eigen::Vector7dAlignedVec vResults;
    double baseNorm = 0;
    std::vector<double> vResultNorms;

    //reset all result norm tallies to 0
    vResultNorms.resize(params.size()*2);
    vResultNorms.assign(params.size()*2,0);

    vParams.resize(params.size()*2);
    vResults.resize(params.size()*2);

    vFunctors.reserve(params.size()*2);


    //prepare the perturbed parameters for all segments
    for(size_t ii = 0 ; ii <  params.size() ; ii++) {
        int plusIdx = ii*2, minusIdx = ii*2+1;
        //perturn this vector by a small amount
        vParams[plusIdx] = params;
        vParams[plusIdx][ii].m_dVal += m_dEpsilon;

        if(g_bUseCentralDifferences == true){
            vParams[minusIdx] = params;
            vParams[minusIdx][ii].m_dVal -= m_dEpsilon;
        }
        //dout("jacobian on dimension " << ii << " is with params:" << vParams[ii*2] << " and " << vParams[ii*2+1]);
    }

    JtJ = Eigen::MatrixXd::Zero(params.size(),params.size());
    Jb = Eigen::MatrixXd::Zero(params.size(),1);

    //dout("JtJ " << JtJ << " Jb " << Jb);
    for(size_t jj = 0; jj < m_vSegmentIndices.size() ; jj++){
        int startIndex = m_vSegmentIndices[jj].first;
        int endIndex = m_vSegmentIndices[jj].second;

        //get the command history for these functors if possible by working back from the current
        //command and adding them to the command list
        CommandList normalCommands = plan.GetDelayedCommandList(MAX_CONTROL_DELAY,startIndex);

        int worldCounter = 0;

        for(size_t ii = 0 ; ii < params.size() ; ii++) {
            //if we have no more worlds, wait until all worlds have finished
            if(worldCounter >= functor.GetCarModel()->GetWorldCount()) {
              while(m_ThreadPool.busy_threads() > 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
              }
                worldCounter  = 0;
            }

            //perturn this vector by a small amount
            std::shared_ptr<ApplyParametersThreadFunctor> functorPlus =
                    std::make_shared<ApplyParametersThreadFunctor>(this,
                                                                   functor,
                                                                   plan,
                                                                   vParams[ii*2],
                                                                   worldCounter,
                                                                   vResults[ii*2],
                                                                   startIndex,
                                                                   endIndex,
                                                                   &normalCommands);
            m_ThreadPool.enqueue(*functorPlus);
            vFunctors.push_back(functorPlus);

            worldCounter++;

            if(g_bUseCentralDifferences == true){
                //if we have no more worlds, wait until all worlds have finished
                if(worldCounter >= functor.GetCarModel()->GetWorldCount()) {
                  while(m_ThreadPool.busy_threads() > 0){
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                  }
                  worldCounter  = 0;
                }

                std::shared_ptr<ApplyParametersThreadFunctor> functorMinus =
                        std::make_shared<ApplyParametersThreadFunctor>(this,
                                                                       functor,
                                                                       plan,
                                                                       vParams[ii*2+1],
                                                                       worldCounter,
                                                                       vResults[ii*2+1],
                                                                       startIndex,
                                                                       endIndex,
                                                                       &normalCommands);
                m_ThreadPool.enqueue(*functorMinus);
                vFunctors.push_back(functorMinus);

                worldCounter++;
            }

        }

        //if we have no more worlds, wait until all worlds have finished
        if(worldCounter >= functor.GetCarModel()->GetWorldCount()) {
          while(m_ThreadPool.busy_threads() > 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          worldCounter  = 0;
        }

        //schedule the calculation of the base error
        std::shared_ptr<ApplyParametersThreadFunctor> functorBase =
                std::make_shared<ApplyParametersThreadFunctor>(this,
                                                               functor,
                                                               plan,
                                                               params,
                                                               worldCounter,
                                                               dBaseError,
                                                               startIndex,
                                                               endIndex,
                                                               &normalCommands);
        m_ThreadPool.enqueue(*functorBase);
        vFunctors.push_back(functorBase);

        //wait for all threads to finish
        while(m_ThreadPool.busy_threads() > 0){
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        //add to the base norm tally (with weight)
        baseNorm += dBaseError.norm();

        Eigen::MatrixXd J = Eigen::MatrixXd(vResults[0].size(), params.size());
        for(size_t ii = 0 ; ii < params.size() ; ii++) {
            int plusIdx = ii*2, minusIdx = ii*2+1;
            if(g_bUseCentralDifferences == true){
                J.col(ii) = (vResults[plusIdx] - vResults[minusIdx])/(2*m_dEpsilon);
                //add to the parameter error tallies (include weight in the norm)
                vResultNorms[plusIdx] += (vResults[plusIdx]).norm();
                vResultNorms[minusIdx] += (vResults[minusIdx]).norm();
            }else{
                vResultNorms[plusIdx] += (vResults[plusIdx]).norm();
                J.col(ii) = (vResults[plusIdx] - dBaseError)/(m_dEpsilon);
            }

            if(J.col(ii).norm() == 0){
                DLOG(INFO) << "Jacobian column for parameter " << ii << "(" << params[ii].m_sName << ") for segment " << jj << " is zero.";
            }
        }



        //cauchy norm

        JtJ += J.transpose()*J;
        Jb += J.transpose()*dBaseError;

        vFunctors.clear();
        //dout("J  " << J.format(CleanFmt) << " JtJ " << JtJ.format(CleanFmt) << " Jb " << Jb.transpose().format(CleanFmt) << "dBaseError" << dBaseError.transpose().format(CleanFmt));
    }

    //by default set the coordinate descent values to the base.
    //this means we're in a local minimum if no other coordinate
    //descent yields a lower error
    dBestNorm = baseNorm;
    vBestParams = params;
    nBestDimension = -1;

    //now that we have JtJ and Jb, calculate the coordinate descent
    //values
    for(int ii = 0, s = params.size() ; ii < s ; ii++) {
        double norm = vResultNorms[ii*2];
        if(norm < dBestNorm) {
            dBestNorm = norm;
            vBestParams = vParams[ii*2];
            nBestDimension = ii;
        }

        if(g_bUseCentralDifferences == true){
            norm = vResultNorms[ii*2+1];
            if(norm < dBestNorm) {
                dBestNorm = norm;
                vBestParams = vParams[ii*2+1];
                nBestDimension = ii;
            }
        }
    }
}




