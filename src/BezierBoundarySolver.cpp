#include "CarPlanner/BezierBoundarySolver.h"
#include "CVars/CVar.h"

#define CURV_MULT 1.2

using namespace CarPlanner;

static int& g_nAggressivenessDivisor(CVarUtils::CreateGetUnsavedCVar("debug.AggressivenessDivisor", 5,""));


///////////////////////////////////////////////////////////////////////////////
BezierBoundarySolver::BezierBoundarySolver()
{
}


///////////////////////////////////////////////////////////////////////////////
double BezierBoundarySolver::GetCurvature(const BoundaryProblem *pProblem, double dist)
{
    BezierBoundaryProblem* bezierProblem = (BezierBoundaryProblem*)pProblem;

    //go through the distance array to find where we are
    int index = 0;
    for(size_t ii = 0 ; ii < bezierProblem->m_vDistances.size() ; ii++){
        if(bezierProblem->m_vDistances[ii] > dist){
            break;
        }else{
            index = ii;
        }
    }

    //now interpolate the curvatures
    double curvature = 0;
    if(index+1 < (int)bezierProblem->m_vDistances.size()){
        double ratio = (dist - bezierProblem->m_vDistances[index])/(bezierProblem->m_vDistances[index+1]-bezierProblem->m_vDistances[index]);
        curvature = ratio*bezierProblem->m_vCurvatures[index+1] + (1-ratio)*bezierProblem->m_vCurvatures[index];
    }else{
        curvature = bezierProblem->m_vCurvatures[index];
    }
    return curvature;
}

///////////////////////////////////////////////////////////////////////////////
void BezierBoundarySolver::_Get5thOrderBezier(BezierBoundaryProblem *pProblem,const Eigen::Vector4d& params)
{
    //the order of the bezier
    const double n = 5.0;

    //calculate the offsets
    const double a1 = params[0];
    const double b1 = params[1];
    const double a2 = params[2];
    const double b2 = params[3];

    //create the handle x and y arrays

    pProblem->m_xVals = Eigen::VectorXd(n+1);
    pProblem->m_yVals = Eigen::VectorXd(n+1);

    const double kInit = pProblem->m_dStartPose[3] / pProblem->m_dAggressiveness;
    const double tGoal = pProblem->m_dGoalPose[2];
    const double kGoal = pProblem->m_dGoalPose[3] / pProblem->m_dAggressiveness;
    //dout("Getting b-curve with goal curvature of " << kGoal);

    //here we're assuming that tInit is always 0
    //so create a rotation matrix for the end goal
    Eigen::Matrix2d Rgoal;
    const double ct = cos(tGoal);
    const double st = sin(tGoal);
    Rgoal << ct, -st,
             st, ct;

    //set the starting point
    pProblem->m_xVals[0] = pProblem->m_yVals[0] = 0;

    //offset the second point knowing that tInit = 0
    pProblem->m_xVals[1] = a1;
    pProblem->m_yVals[1] = 0;

    //offset the third point using the initial curvature
    double h = kInit*powi(a1,2)*(n/(n+1));
    pProblem->m_xVals[2] = pProblem->m_xVals[1] + b1;
    pProblem->m_yVals[2] = pProblem->m_yVals[1] + h;

    //and now set the end points
    pProblem->m_xVals[5] = pProblem->m_dGoalPose[0];
    pProblem->m_yVals[5] = pProblem->m_dGoalPose[1];

    //calculate the offset
    Eigen::Vector2d offset(-a2,0);
    offset = Rgoal * offset;
    pProblem->m_xVals[4] = pProblem->m_xVals[5] + offset[0];
    pProblem->m_yVals[4] = pProblem->m_yVals[5] + offset[1];

    //calculate the final point using last curvature
    h = kGoal*powi(a2,2)*(n/(n+1.0));
    offset << -b2, -h;
    offset = Rgoal * offset;
    pProblem->m_xVals[3] = pProblem->m_xVals[4] + offset[0];
    pProblem->m_yVals[3] = pProblem->m_yVals[4] + offset[1];

}

///////////////////////////////////////////////////////////////////////////////
void BezierBoundarySolver::Solve(BoundaryProblem *pProblem)
{
    //double time = Tic();
    BezierBoundaryProblem* bezierProblem = (BezierBoundaryProblem*)pProblem;

    //find the distance between the start and finish
    double dist = sqrt(powi(pProblem->m_dGoalPose[0],2) + powi(pProblem->m_dGoalPose[1],2));
    bezierProblem->m_dSegLength = dist/g_nAggressivenessDivisor;
    //first get the guess bezier
    if(bezierProblem->m_bSolved == false){

    }

    bezierProblem->m_dSegLength = std::max(1e-2,std::min(bezierProblem->m_dSegLength,dist/2));

    bezierProblem->m_dParams = Eigen::Vector4d(bezierProblem->m_dSegLength,bezierProblem->m_dSegLength,bezierProblem->m_dSegLength,bezierProblem->m_dSegLength);
    _Get5thOrderBezier(bezierProblem,bezierProblem->m_dParams);

    //and now sample it
    _Sample5thOrderBezier(bezierProblem);

    //now iterate to reduce curvature
    if(bezierProblem->m_bSolved == false){
        //_IterateCurvatureReduction(bezierProblem,bezierProblem->m_dParams);
        //_IterateCurvatureReduction(bezierProblem,bezierProblem->m_dParams);
    }

    //indicate that the problem has been solved
    bezierProblem->m_bSolved = true;

    //dout("2D solve with goal " << bezierProblem->m_dGoalPose.transpose() <<" took " << Toc(time) << " seconds.");
}

///////////////////////////////////////////////////////////////////////////////
void BezierBoundarySolver::_GetCoefs(Eigen::Vector6d& coefs,Eigen::Vector6d&  dCoefs,Eigen::Vector6d&  ddCoefs, const double& t)
{
    Eigen::Vector5d tPowers, omtPowers, tmoPowers;

    //first calculate the 5 powers of t and 1-t
    double omt = 1-t;
    double tmo = t-1;
    tPowers[0] = t;
    omtPowers[0] = omt;
    tmoPowers[0] = tmo;
    for(int ii = 1  ; ii < 5 ; ii++){
        tPowers[ii] = tPowers[ii-1]*t;
        omtPowers[ii] = omtPowers[ii-1]*omt;
        tmoPowers[ii] = tmoPowers[ii-1]*tmo;
    }

    //now construct the bezier coefficients (using pascal's triangle)
    coefs[0] = omtPowers[4];               //(1-t)^5
    coefs[1] = 5*tPowers[0]*omtPowers[3];  //5*t*(1-t)^4
    coefs[2] = 10*tPowers[1]*omtPowers[2]; //10*t^2*(1-t)^3
    coefs[3] = 10*tPowers[2]*omtPowers[1]; //10*t^3*(1-t)^2
    coefs[4] = 5*tPowers[3]*omtPowers[0];  //5*t^4*(1-t)
    coefs[5] = tPowers[4];                 //t^5

    //construct the first derivative bezier coefficients
    dCoefs[0] = -5*tmoPowers[3];                                                //-5*(t - 1)^4
    dCoefs[1] = 20*tPowers[0]*tmoPowers[2] + 5*tmoPowers[3];                    //20*t*(t - 1)^3 + 5*(t - 1)^4
    dCoefs[2] = -20*tPowers[0]*tmoPowers[2] - 30*tPowers[1]*tmoPowers[1];       //- 20*t*(t - 1)^3 - 30*t^2*(t - 1)^2
    dCoefs[3] = 10*tPowers[2]*(2*tPowers[0] - 2) + 30*tPowers[1]*tmoPowers[1];  //10*t^3*(2*t - 2) + 30*t^2*(t - 1)^2
    dCoefs[4] = -20*tPowers[2]*tmoPowers[0] - 5*tPowers[3];                     //- 20*t^3*(t - 1) - 5*t^4
    dCoefs[5] = 5*tPowers[3];                                                   //5*t^4

    //construct the second derivative bezier coefficients
    ddCoefs[0] = -20*tmoPowers[2];                                               //-20*(t - 1)^3
    ddCoefs[1] = 60*tPowers[0]*tmoPowers[1] + 40*tmoPowers[2];                   //60*t*(t - 1)^2 + 40*(t - 1)^3
    ddCoefs[2] = -120*tPowers[0]*tmoPowers[1] - 20*tmoPowers[2]
                 - 30*tPowers[1]*(2*tPowers[0] - 2);                             //- 120*t*(t - 1)^2 - 20*(t - 1)^3 - 30*t^2*(2*t - 2)
    ddCoefs[3] = 60*tPowers[0]*tmoPowers[1] + 60*tPowers[1]*(2*tPowers[0] - 2)
            + 20*tPowers[2];                                                     //60*t*(t - 1)^2 + 60*t^2*(2*t - 2) + 20*t^3
    ddCoefs[4] = -60*tPowers[1]*tmoPowers[0] - 40*tPowers[2];                    //- 60*t^2*(t - 1) - 40*t^3
    ddCoefs[5] = 20*tPowers[2];                                                 //20*t^3
}

///////////////////////////////////////////////////////////////////////////////
void BezierBoundarySolver::_Sample5thOrderBezier(BezierBoundaryProblem* pProblem)
{
    Eigen::Vector6d coefs, dCoefs, ddCoefs;
    double dt = 1.0/pProblem->m_nDiscretization;

    pProblem->m_vCurvatures.clear();
    pProblem->m_vDistances.clear();
    pProblem->m_vPts.clear();

    pProblem->m_vCurvatures.reserve(pProblem->m_nDiscretization+1);
    pProblem->m_vDistances.reserve(pProblem->m_nDiscretization+1);
    pProblem->m_vPts.reserve(pProblem->m_nDiscretization+1);
    pProblem->m_dDistance = 0;

    Eigen::Vector2d lastPt(pProblem->m_xVals[0],pProblem->m_yVals[0]);
    double t = 0;

    for(int ii = 0 ; ii <= pProblem->m_nDiscretization ; ii++) {
        _GetCoefs(coefs,dCoefs,ddCoefs,t);

        //calculate the x and y position of the curve at this t
        pProblem->m_vPts.push_back(Eigen::Vector2d(coefs.transpose()*pProblem->m_xVals, coefs.transpose()*pProblem->m_yVals));
        pProblem->m_dDistance += (lastPt - pProblem->m_vPts.back()).norm();
        lastPt = pProblem->m_vPts.back();

        //calculate the derivatives of x and y
        double dX = dCoefs.transpose()*pProblem->m_xVals;
        double ddX = ddCoefs.transpose()*pProblem->m_xVals;

        double dY = dCoefs.transpose()*pProblem->m_yVals;
        double ddY = ddCoefs.transpose()*pProblem->m_yVals;

        //and now calculate the curvature
        double sq = sqrt(powi(dX*dX + dY*dY,3));  //(dX^2+dY^2)^(3/2)
        double curvature = (dX*ddY-dY*ddX)/sq;
        pProblem->m_vCurvatures.push_back(curvature*pProblem->m_dAggressiveness);  //k = (dX*ddY - dY*ddX)/((dX^2 + dY^2)^(3/2))
        pProblem->m_vDistances.push_back(pProblem->m_dDistance);

        t += dt;
    }
    //dout("Sampling bezier. Final point " << pProblem->m_vPts.back().transpose());
}

///////////////////////////////////////////////////////////////////////////////
double BezierBoundarySolver::_GetMaximumCurvature(const BezierBoundaryProblem* pProblem)
{
    double maxK = DBL_MIN;
    //go through the list of curvatures and return the maximum
    for(int ii = 0, s = pProblem->m_vCurvatures.size() ; ii < s ; ii++)
    {
        maxK = std::max(maxK,pProblem->m_vCurvatures[ii]);
    }
    return maxK;
}

///////////////////////////////////////////////////////////////////////////////
void BezierBoundarySolver::_IterateCurvatureReduction(BezierBoundaryProblem* pProblem,Eigen::Vector4d& params)
{
    double epsilon = 0.0001;
    //create a jacobian for the parameters by perturbing them
    Eigen::Vector4d Jt; //transpose of the jacobian
    BezierBoundaryProblem origProblem = *pProblem;
    double maxK = _GetMaximumCurvature(pProblem);
    for(int ii = 0; ii < 4 ; ii++){
        Eigen::Vector4d epsilonParams = params;
        epsilonParams[ii] += epsilon;
        _Get5thOrderBezier(pProblem,epsilonParams);
        _Sample5thOrderBezier(pProblem);
        double kPlus = _GetMaximumCurvature(pProblem);

        epsilonParams[ii] -= 2*epsilon;
        _Get5thOrderBezier(pProblem,epsilonParams);
        _Sample5thOrderBezier(pProblem);
        double kMinus = _GetMaximumCurvature(pProblem);
        Jt[ii] = (kPlus-kMinus)/(2*epsilon);
    }

    //now that we have Jt, we can calculate JtJ
    Eigen::Matrix4d JtJ = Jt*Jt.transpose();
    //thikonov regularization
    JtJ += Eigen::Matrix4d::Identity();

    Eigen::Vector4d deltaParams = JtJ.inverse() * Jt*maxK;
    params -= deltaParams;
    _Get5thOrderBezier(pProblem,params);
    _Sample5thOrderBezier(pProblem);
    //double finalMaxK = _GetMaximumCurvature(pProblem);

    //dout("2D Iteration took k from " << maxK << " to " << finalMaxK);
}


