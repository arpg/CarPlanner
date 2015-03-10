#include "DirectBoundarySolver.h"

DirectBoundarySolver::DirectBoundarySolver()
{
}

///////////////////////////////////////////////////////////////////////////////
void DirectBoundarySolver::Solve(BoundaryProblem *pProblem)
{
    //DirectBoundaryProblem* directProblem = (DirectBoundaryProblem*)pProblem;

    //first find the distance to the goal
    //double dDist = (directProblem->m_dGoalPose.head(2) - directProblem->m_dStartPose.head(2)).norm();

    //now split this in 3 to calculate the turning radii
    //double dRad1 = dDist/3;
    //double dRad2 = dDist/3;

    //place the center of the circles perpendicular to the travel direction
//    Eigen::Vector2d dDir(sin(directProblem->m_dStartPose(2)),cos(directProblem->m_dStartPose(2)));
//    Eigen::Vector2d dCircCenter1 = directProblem->m_dStartPose.head(2) + dDir*dRad1;
//    dDir = Eigen::Vector2d(sin(directProblem->m_dGoalPose(2)),cos(directProblem->m_dGoalPose(2)));
//    Eigen::Vector2d dCircCenter2 = directProblem->m_dGoalPose.head(2) + dDir*dRad1;

    //calcuate the
}
