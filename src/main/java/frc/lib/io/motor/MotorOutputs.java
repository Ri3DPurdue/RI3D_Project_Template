package frc.lib.io.motor;

public record MotorOutputs(
    double position,
    double velocity,
    double voltage,
    double statorCurrent,
    double supplyCurrent,
    double temperatureCelsius
) {
    public MotorOutputs() {
        this(0, 0, 0, 0, 0, 0);
    }
}