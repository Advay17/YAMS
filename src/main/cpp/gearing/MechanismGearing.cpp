#include "gearing/MechanismGearing.h"

MechanismGearing::MechanismGearing(GearBox gearBox) : gearBox(gearBox) {}
MechanismGearing::MechanismGearing(GearBox gearBox, Sprocket sprockets) : gearBox(gearBox), sprockets(sprockets) {}
double MechanismGearing::getRotorToMechanismRatio() const{
    double ratio = gearBox.getInputToOutputConversionFactor();
    if (sprockets.has_value()) {
        ratio *= sprockets->getInputToOutputConversionFactor();
    }
    return ratio;
}
double MechanismGearing::getMechanismToRotorRatio() const {
    double ratio = gearBox.getOutputToInputConversionFactor();
    if (sprockets.has_value()) {
        ratio *= sprockets->getOutputToInputConversionFactor();
    }
    return ratio;
}