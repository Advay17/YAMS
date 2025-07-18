#include <cstdarg>
#include "math/SmartMath.h"
double SmartMath::SensorToMechanismRatio(double stages...){
    if(stages == 0){
        //TODO: NoStagesGivenException
    }
    va_list args;
    va_start(args, stages);
    double sensorToMechanismRatio = 1;
    for(int i=0; i < stages; i++){
        double stage = va_arg(args, double);
        sensorToMechanismRatio *= (1 / stage);
    }
    return sensorToMechanismRatio;
}

double GearBox(double stages...){
    if(stages == 0){
        //TODO: NoStagesGivenException
    }
    va_list args;
    va_start(args, stages);
    double gearBox = 1;
    for(int i=0; i < stages; i++){
        double stage = va_arg(args, double);
        gearBox *= (1 / stage);
    }
    return gearBox;
}