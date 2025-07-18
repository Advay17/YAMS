#include "gearing/MechanismGearing.h"

MechanismGearing::MechanismGearing(GearBox gearBox) : gearBox(gearBox) {}
MechanismGearing::MechanismGearing(GearBox gearBox, Sprocket sprockets) : gearBox(gearBox), sprockets(sprockets) {}
double MechanismGearing::GetRotorToMechanismRatio() const{
    double ratio = gearBox.GetInputToOutputConversionFactor();
    if (sprockets.has_value()) {
        ratio *= sprockets->GetInputToOutputConversionFactor();
    }
    return ratio;
}
double MechanismGearing::GetMechanismToRotorRatio() const {
    double ratio = gearBox.GetOutputToInputConversionFactor();
    if (sprockets.has_value()) {
        ratio *= sprockets->GetOutputToInputConversionFactor();
    }
    return ratio;
}