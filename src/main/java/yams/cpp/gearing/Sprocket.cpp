#include <Sprocket.h>
#include <vector>
Sprocket::Sprocket(std::span<double> sprocketReductionStage) {
    setupStages(sprocketReductionStage);
}
Sprocket::Sprocket(std::span<std::string> reductionStage){
    std::vector<double> stages(reductionStage.size());
    for (size_t i = 0; i < sizeof(reductionStage) / sizeof(reductionStage[0]); i++)
    {
        std::string stage = reductionStage[i];
        if (!stage.contains(":"))
        {
            // TODO: InvalidStageGivenException
        }
        double in = std::stod(stage.substr(0, stage.find(":")));
        double out = std::stod(stage.substr(stage.find(":") + 1));
        stages[i] = in / out;
    }
    setupStages(stages);
}

void Sprocket::setupStages(std::span<double> sprocketReductionStage){
    reductionStages = sprocketReductionStage;
        if (reductionStages.size() == 0)
    {
      //TODO: NoStagesGivenException()
    }
    double sprocketRatio = (1 / reductionStages[0]);
    for (int i = 1; i < reductionStages.size(); i++)
    {
      sprocketRatio *= (1 / reductionStages[i]);
    }
    sprocketReductionRatio = sprocketRatio;
}

double Sprocket::getInputToOutputConversionFactor() const {
    return sprocketReductionRatio;
}

double Sprocket::getOutputToInputConversionFactor() const {
    return 1.0 / sprocketReductionRatio;
}