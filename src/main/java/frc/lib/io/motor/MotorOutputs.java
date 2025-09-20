package frc.lib.io.motor;

public record MotorOutputs(
    double position,
    double velocity,
    double voltage
) {
    public MotorOutputs() {
        this(0, 0, 0);
    }
}