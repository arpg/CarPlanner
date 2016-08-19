#ifndef DIRECTBOUNDARYSOLVER_H
#define DIRECTBOUNDARYSOLVER_H

#include "CarPlannerCommon.h"
#include "BoundarySolver.h"
#include "float.h"

struct DirectBoundaryProblem : BoundaryProblem
{
    DirectBoundaryProblem() {}
    std::vector<std::tuple<double,double> > m_vParams;
};

class DirectBoundarySolver
{
public:
    DirectBoundarySolver();
    void Solve(BoundaryProblem *pProblem);
};

#endif // DIRECTBOUNDARYSOLVER_H
