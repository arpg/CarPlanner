#include "CarPlanner/CarParameters.h"
#include "CarPlanner/BulletCarModel.h"
#include "boost/lexical_cast.hpp"

////////////////////////////////////////////////////////////////
const char * const CarParameters::Names[] = {"WheelBase", "Width", "Height", "DynamicFrictionCoef",
                                             "StaticSideFrictionCoef", "SlipCoefficient", "ControlDelay",
                                             "Mass", "WheelRadius", "WheelWidth","TractionFriction",
                                             "SuspConnectionHeight","Stiffness","MaxSuspForce","MaxSuspTravel",
                                             "SuspRestLength","CompDamping","ExpDamping","RollInfluence",
                                             "SteeringCoef","MaxSteering","MaxSteeringRate","AccelOffset",
                                             "SteeringOffset","StallTorqueCoef","TorqueSpeedSlope",
                                             "Magic_B","Magic_C","Magic_E"};

////////////////////////////////////////////////////////////////
bool CarParameters::SaveToFile(const std::string sFile,const CarParameterMap& map)
{
    dout("Writing parameter map to " << sFile << "-----------------");
    std::ofstream file;
    file.open(sFile);
    for(const CarParameterPair& pair: map ){
        char cLine[256];
        sprintf(cLine,"%s,%.9f\n",Names[pair.first],pair.second);
        file.write(cLine,strlen(cLine));
    }
    file.close();
    return true;
}

////////////////////////////////////////////////////////////////
bool CarParameters::LoadFromFile(const std::string sFile,CarParameterMap& map)
{
    dout("Reading parameter map from " << sFile << "-----------------");
    std::ifstream file;
    //open the file for read
    file.open(sFile.c_str());
    std::string line;
    while(std::getline(file,line))
    {
        std::vector<std::string> vals;
        std::stringstream  lineStream(line);
        std::string        cell;
        while(std::getline(lineStream,cell,','))
        {
            vals.push_back(cell.c_str());
        }

        if(vals.size() == 2){
            //find the name
            bool bFound = false;
            for(size_t ii = 0; ii < sizeof(Names) ; ii++){
                if(vals[0].compare(std::string(Names[ii])) == 0){
                    bFound = true;
                    double val = boost::lexical_cast<double>(vals[1]);
                    map[ii] = val;
                    dout("Loading parameter " << Names[ii] << " with value " << val);
                    break;
                }
            }

            if(bFound == false){
                dout("ERROR - parameter " << vals[0] << " not recognized.");
                return false;
            }
        }else{
            dout("ERROR - line with less than 2 comma delimited items found when parsing parameters.");
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////
void CarParameters::PrintAllParams(const CarParameterMap &map)
{
    dout("Printing parameter map -----------------");
    for(const CarParameterPair& pair: map ){
        dout(Names[pair.first] << ":" << pair.second);
    }
}

////////////////////////////////////////////////////////////////
RegressionParameter::RegressionParameter(CarParameterMap& map, int nKey, BulletCarModel* pModel/* = NULL*/):
    m_dVal(map[nKey]),
    m_nKey(nKey),
    m_sName(CarParameters::Names[nKey]),
    m_pModel(pModel){}

////////////////////////////////////////////////////////////////
bool RegressionParameter::AreEqual(const std::vector<RegressionParameter>& params1, const std::vector<RegressionParameter>& params2)
{
    for(size_t ii = 0 ; ii < params1.size() ; ii++) {
        if(params1[ii].m_dVal != params2[ii].m_dVal){
            return false;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////
void RegressionParameter::UpdateValue(const double newVal)
{
    m_dVal = newVal;
    std::vector<RegressionParameter> params = {*this};
    m_pModel->UpdateParameters(params);
}
