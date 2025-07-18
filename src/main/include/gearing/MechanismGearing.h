#include "GearBox.h"
#include <optional>
#include "Sprocket.h"
class MechanismGearing
{
public:
    MechanismGearing(GearBox gearBox);
    MechanismGearing(GearBox gearBox, Sprocket sprockets);
    double GetRotorToMechanismRatio() const;
    double GetMechanismToRotorRatio() const;

private:
    const GearBox gearBox;
    std::optional<Sprocket> sprockets = {};

};