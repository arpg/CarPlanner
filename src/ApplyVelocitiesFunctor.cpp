#include "CarPlanner/ApplyVelocitiesFunctor.h"
#include "CarPlanner/LocalPlanner.h"
#include <Eigen/StdVector>

static bool g_bSkidCompensationActive = false;

////////////////////////////////////////////////////////////////
ApplyVelocitesFunctor5d::ApplyVelocitesFunctor5d(BulletCarModel *pCarModel, Eigen::Vector3d dInitTorques, CommandList *pPreviousCommands /* = NULL */) :
        m_pCarModel(pCarModel),
        m_dInitTorques(dInitTorques),
        m_bNoDelay(false)
{
  pangolin::Var<bool>::Attach("debug.SkidCompensationActive", g_bSkidCompensationActive);
  if(pPreviousCommands != NULL) {
    m_lPreviousCommands = *pPreviousCommands;
  }
}

////////////////////////////////////////////////////////////////
double ApplyVelocitesFunctor5d::GetGravityCompensation(int nIndex)
{
  double aExtra =  -(m_pCarModel->GetTotalGravityForce(m_pCarModel->GetWorldInstance(nIndex))/m_pCarModel->GetWorldInstance(nIndex)->m_Parameters[CarParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/;
  return aExtra;
}

////////////////////////////////////////////////////////////////
double ApplyVelocitesFunctor5d::GetFrictionCompensation(int nIndex, double dt)
{
  double aExtra = -m_pCarModel->GetTotalWheelFriction(nIndex,dt)/m_pCarModel->GetWorldInstance(nIndex)->m_Parameters[CarParameters::Mass];
  return aExtra;
}

////////////////////////////////////////////////////////////////
//double ApplyVelocitesFunctor5d::GetSteeringCompensation(VehicleState& state, double phi, double curvature, int nIndex)
//{
//    //get the corrected steering parameters
//    phi = m_pCarModel->GetCorrectedSteering(curvature,nIndex);
//    double vf = state.m_dV.norm();
//    double aExtra = fabs(vf*vf*curvature*tan(phi))*CAR_STEERING_COMPENSATION_COEFFICIENT;
//    return aExtra;
//}

////////////////////////////////////////////////////////////////
void ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& startingState,
                                              std::vector<ControlCommand>& vCommands,
                                              std::vector<VehicleState>& vStatesOut,
                                              const int nStartIndex,
                                              const int nEndIndex,
                                              const int nIndex,
                                              const bool bNoCompensation /*= false*/,
                                              const CommandList *pPreviousCommands /*= NULL*/) {
  Eigen::Vector3d torques;
  Eigen::Vector4dAlignedVec vCoefs;
  BulletWorldInstance* pWorld = m_pCarModel->GetWorldInstance(nIndex);

  vStatesOut.clear();

  double dTime = 0;

  VehicleState currentState;
  m_pCarModel->SetState(nIndex,startingState);
  m_pCarModel->GetVehicleState(nIndex,currentState);
  VehicleState* pCurrentState = &currentState; //this is necessary as we need to get a pointer to the current state for compensations
  //clear all the previous commands but chose between the member list or the one passed to the function
  /// Running the next command sometimes leads to a race condition.
  /// Using debug mode and stepping through solves the problem.
  // Try reformulating such that worldId isnt handed off, but rather the entire world is; lock the world.
  m_pCarModel->SetCommandHistory(nIndex, pPreviousCommands == NULL ? m_lPreviousCommands : *pPreviousCommands);

  //m_pCarModel->ResetCommandHistory(nIndex);

  vStatesOut.resize(nEndIndex-nStartIndex);

  ControlCommand command;
  for (int ii = nStartIndex; ii < nEndIndex; ii++) {
    //update the vehicle state
    //approximation for this dt
    command = vCommands[ii];
    if(bNoCompensation == false ){
      //HACK: SampleAcceleration actually returns this as acceleration and
      //not force, so we have to change that here
      double totalAccel = command.m_dForce;



      //compensate for gravity/slope
      double aExtra = 0;
      double dCorrectedCurvature;
      command.m_dPhi = m_pCarModel->GetSteeringAngle(command.m_dCurvature,
                                                     dCorrectedCurvature,nIndex,1.0);

      //if(dRatio < 1.0){
      if(g_bSkidCompensationActive){
        //get the steering compensation
        std::pair<double,double> leftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nIndex,0,command.m_dPhi,command.m_dT);
        std::pair<double,double> rightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nIndex,1,command.m_dPhi,command.m_dT);
        double dRatio = std::max(fabs(leftWheel.second/leftWheel.first),fabs(rightWheel.second/rightWheel.first));

        for(int ii = 0 ; ii < 5 && dRatio < 1.0 ; ii++){
          command.m_dCurvature *=1.5;///= (dRatio);
          command.m_dPhi = m_pCarModel->GetSteeringAngle(command.m_dCurvature,
                                                         dCorrectedCurvature,nIndex,1.0);
          std::pair<double,double> newLeftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nIndex,0,command.m_dPhi,command.m_dT);
          std::pair<double,double> newRightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nIndex,1,command.m_dPhi,command.m_dT);
          dRatio = std::max(fabs(newLeftWheel.second/leftWheel.first),fabs(newRightWheel.second/rightWheel.first));
        }
      }



      aExtra += GetGravityCompensation(nIndex);
      //aExtra += GetSteeringCompensation(*pCurrentState,command.m_dPhi,command.m_dCurvature,nIndex);
      aExtra += GetFrictionCompensation(nIndex,command.m_dT);


      totalAccel += aExtra;

      //            if(dRatio < 1.0){
      //                totalAccel = 0;//(dRatio*dRatio*dRatio);
      //            }

      //actually convert the accel (up to this point) to a force to be applied to the car
      command.m_dForce = totalAccel*m_pCarModel->GetWorldInstance(nIndex)->m_Parameters[CarParameters::Mass];
      //here Pwm = (torque+slope*V)/Ts
      command.m_dForce = sgn(command.m_dForce)* (fabs(command.m_dForce) + pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*pWorld->m_state.m_dV.norm())/pWorld->m_Parameters[CarParameters::StallTorqueCoef];
      command.m_dForce += pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;

      //offset and coef are in 0-1 range, so multiplying by SERVO_RANGE is necessary
      command.m_dPhi = SERVO_RANGE*(command.m_dPhi*pWorld->m_Parameters[CarParameters::SteeringCoef] +
                      pWorld->m_Parameters[CarParameters::SteeringOffset]);

      //save the command changes to the command array -- this is so we can apply
      //the commands to the vehicle WITH compensation
      vCommands[ii] = command;
    }

    //set the timestamp for this command
    vCommands[ii].m_dTime = dTime;
    dTime += command.m_dT;
    m_pCarModel->UpdateState(nIndex,command,command.m_dT,m_bNoDelay);
    m_pCarModel->GetVehicleState(nIndex,vStatesOut[ii-nStartIndex]);
    vStatesOut[ii-nStartIndex].m_dCurvature = command.m_dCurvature;
    pCurrentState = &vStatesOut[ii-nStartIndex];
    pCurrentState->m_dTime = dTime;

  }
}

////////////////////////////////////////////////////////////////
VehicleState ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nIndex /*= 0*/,
                                                      bool noCompensation /*= false*/) {
  ApplyVelocities(startState,
                  sample.m_vCommands,
                  sample.m_vStates,
                  0,
                  sample.m_vCommands.size(),
                  nIndex,
                  noCompensation,
                  NULL);
  return sample.m_vStates.back();
}






