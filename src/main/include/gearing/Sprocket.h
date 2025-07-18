#include <span>
#include <string>
class Sprocket
{
public:
    /**
     * Create the sprocket given the teeth of each sprocket in the chain.
     *
     * @param sprocketReductionStage Sprocket teeth, in the form of "IN:OUT" => IN/OUT
     */
    Sprocket(std::span<double> sprocketReductionStage);

    /**
     * Construct the {@link Sprocket} with the reduction stages given.
     *
     * @param reductionStage List of stages in the format of "IN:OUT".
     */
    Sprocket(std::span<std::string> sprocketReductionStage);
    /**
     * Get the conversion factor to transform the sprocket input into the sprocket output rotations.
     *
     * @return OUT/IN or OUT:IN
     */
    double GetInputToOutputConversionFactor() const;
    /**
     * Get the conversion factor to transform the sprocket output value into the sprocket input value.
     *
     * @return IN:OUT or IN/OUT
     */
    double GetOutputToInputConversionFactor() const;

private:
    /**
     * Stages in the Sprocket chain.
     */
    std::span<double> reductionStages;
    /**
     * The input to output conversion factor.
     */
    double sprocketReductionRatio;
    /**
     * Set up the reduction stages for the {@link Sprocket}
     *
     * @param sprocketReductionStage Reductions in the form of "IN:OUT" => IN/OUT
     */
    void setupStages(std::span<double> sprocketReductionStage);
};