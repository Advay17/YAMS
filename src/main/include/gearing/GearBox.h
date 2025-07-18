#include <string>
#include <span>
class GearBox
{
public:
    /**
     * Construct the {@link GearBox} with the reduction stages given.
     *
     * @param reductionStage Reduction stages where the number is > 0 to indicate a reduction.
     */
    GearBox(std::span<double> reductionStage);
    /**
     * Construct the {@link GearBox} with the reduction stages given.
     *
     * @param reductionStage List of stages in the format of "IN:OUT".
     */
    GearBox(std::span<std::string> reductionStage);

    /**
     * Get the conversion factor to transform the gearbox input into the gear box output rotations.
     *
     * @return OUT/IN or OUT:IN
     */
    double GetInputToOutputConversionFactor() const;
    /**
     * Get the conversion factor to transform the gearbox output value into the gear box input value.
     *
     * @return IN:OUT or IN/OUT
     */
    double GetOutputToInputConversionFactor() const;

private:
    /**
     * Stages in the gear box
     */
    std::span<double> reductionStages;
    /**
     * Conversion factor of the gearbox from input to output.
     */
    double gearReductionRatio;
    /**
     * Sets the stages and calculates the reduction for the {@link GearBox}
     *
     * @param reductionStage Reduction stages where the number is > 0 to indicate a reduction.
     */
    void setupGearBox(std::span<double> reductionStage);
};