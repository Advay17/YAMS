class SmartMath
{
public:
    /**
     * Create the sensor to mechanism ratio.
     *
     * @param stages
     * @return
     */
    static double sensorToMechanismRatio(double stages...);
    /**
     * Create the gear ratio based off of the stages in the gear box.
     *
     * @param stages stages between the motor and output shaft.
     * @return rotor rotations to mechanism ratio in the form of MECHANISM_ROTATIONS/ROTOR_ROTATIONS or
     * ROTOR_ROTATIONS:MECHANISM_ROTATIONS
     */
    static double gearBox(double stages...);
};