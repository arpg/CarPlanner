#ifndef CARPARAMETERS_H
#define CARPARAMETERS_H

#include "CarPlannerCommon.h"

//enum CarParameterOrder
//{
//    //eSteeringCoef,
//    eWheelBase,
//    //eAccelCoef,
//    //eMass,
//    eFrictionCoef,
//    eControlDelay
//};

typedef std::pair<int,double> CarParameterPair;
typedef std::map<int, double> CarParameterMap;

enum ControlTarget
{
    eTargetSimulation = 0,
    eTargetExperiment = 1
};




/// Structure containing all parameters for a car
class CarParameters
{
public:


    enum{
        //body options
         WheelBase = 0,                //< Wheel base of the car
         Width = 1,                    //< Width of the car
         Height = 2,                   //< Height of the CG above the vehicle datum
         DynamicFrictionCoef = 3,             //< Friction coefficient, which slows the car down.
         StaticSideFrictionCoef = 4,            //< Static side friction coefficient which enforces nonholonomity
         SlipCoefficient = 5,
         ControlDelay = 6,             //< Control delay of the vehicle (in seconds)
         Mass = 7,

        //wheel options
         WheelRadius = 8,              //< Radius of the wheels
         WheelWidth = 9,               //< Thickness of the wheels
         TractionFriction = 10,         //< Friction value of the wheels to the ground

        //suspension options
        //reference : https://docs.google.com/document/edit?id=18edpOwtGgCwNyvakS78jxMajCuezotCU_0iezcwiFQc&pli=1
         SuspConnectionHeight = 11,    //< Height of the suspension connection point above the vehicle datum
         Stiffness = 12,               //< Suspension spring stiffness
         MaxSuspForce = 13,            //< Maximum allowable suspension force, in Newtons
         MaxSuspTravel = 14,           //< Maximum allowable travel distance for the suspension from the rest length
         SuspRestLength = 15,          //< Rest length of the suspension. This should be the point where the suspension lies when there is no contact
        /// The damping coefficient for when the suspension is compressed. Set to k * 2.0 * btSqrt(m_suspensionStiffness)
        /// so k is proportional to critical damping. k = 0.0 undamped & bouncy, k = 1.0 critical damping
        /// k = 0.1 to 0.3 are good values
         CompDamping = 16,
        /// The damping coefficient for when the suspension is expanding.  See the comments for m_wheelsDampingCompression for how to set k.
        /// m_wheelsDampingRelaxation should be slightly larger than m_wheelsDampingCompression, eg k = 0.2 to 0.5
         ExpDamping = 17,
        /// Reduces the rolling torque applied from the wheels that cause the vehicle to roll over.
        /// This is a bit of a hack, but it's quite effective. 0.0 = no roll, 1.0 = physical behaviour.
        /// If m_frictionSlip is too high, you'll need to reduce this to stop the vehicle rolling over.
        /// You should also try lowering the vehicle's centre of mass
         RollInfluence = 18,

         //AccelCoef = 19,                //< Multiplier coefficient for acceleration
         SteeringCoef = 19,             //< Multiplier coefficient for steering
         MaxSteering = 20,
         MaxSteeringRate = 21,

         AccelOffset = 22,              //< Offset from the center for acceleration
         SteeringOffset = 23,           //< Offset from the center for steering

        ///DC Motor coefficients
        StallTorqueCoef = 24,
        TorqueSpeedSlope = 25,

        //magic formula params
        MagicFormula_B  = 26,
        MagicFormula_C  = 27,
        MagicFormula_E = 28     //magic formula D is calculated as Fz * StaticSideFrictionCoef
    };

    static const char * const Names[];


    static bool LoadFromFile(const std::string sFile, CarParameterMap &map);
    static void PrintAllParams(const CarParameterMap &map);
    static bool SaveToFile(const std::string sFile, const CarParameterMap &map);
};

class BulletCarModel;

struct RegressionParameter
{
    RegressionParameter(CarParameterMap& map, int nKey, BulletCarModel* pModel = NULL);
    static bool AreEqual(const std::vector<RegressionParameter>& params1, const std::vector<RegressionParameter>& params2);
    void UpdateValue(const double newVal);
    double m_dVal;
    int m_nKey;
    std::string m_sName;
    BulletCarModel* m_pModel;
};

inline std::ostream& operator<<( std::ostream& Stream, RegressionParameter& parameter )
{
    Stream << parameter.m_dVal;
    return Stream;
}

inline std::istream& operator>>( std::istream& Stream, RegressionParameter& parameter )
{
    double val;
    Stream >> val;
    parameter.UpdateValue(val);
    return Stream;
}

inline std::ostream& operator<<( std::ostream& Stream, const std::vector<RegressionParameter>& params )
{
    Stream << "[ ";

    for( unsigned int ii = 0; ii < params.size(); ii++ ) {
        Stream << params[ii].m_sName << ":";
        Stream << params[ii].m_dVal;
        Stream << ", ";
    }

    Stream << " ]";

    return Stream;
}



#endif // CARPARAMETERS_H
