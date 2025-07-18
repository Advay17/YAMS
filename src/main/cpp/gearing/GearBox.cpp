
#include <vector>
#include "gearing/GearBox.h"
#include <wpi/StringExtras.h>
GearBox::GearBox(std::span<double> reductionStage)
{
    setupGearBox(reductionStage);
}
GearBox::GearBox(std::span<std::string> reductionStage)
{
    std::vector<double> stages(reductionStage.size());
    for (size_t i = 0; i < reductionStage.size(); i++)
    {
        std::string stage = reductionStage[i];
        if (wpi::contains(stage, ":"))
        {
            // TODO: InvalidStageGivenException
        }
        double in = std::stod(stage.substr(0, stage.find(":")));
        double out = std::stod(stage.substr(stage.find(":") + 1));
        stages[i] = in / out;
    }
    setupGearBox(stages);
}

void GearBox::setupGearBox(std::span<double> reductionStage)
{
    reductionStages = reductionStage;
    if (reductionStages.size() == 0)
    {
        // TODO: NoStagesGivenException
    }
    double gearBox = 1 / reductionStages[0];
    for (int i = 1; i < reductionStages.size(); i++)
    {
        gearBox *= (1 / reductionStages[i]);
    }
    gearReductionRatio = gearBox;
}

double GearBox::GetInputToOutputConversionFactor() const
{
    return gearReductionRatio;
}

double GearBox::GetOutputToInputConversionFactor() const
{
    return 1.0 / gearReductionRatio;
}