package frc.lib.io.motor;

public class MotorOutputs { 
    public double position;
    public double velocity;
    public double voltage;
    public double statorCurrent;
    public double supplyCurrent;
    public double temperatureCelsius;
    public MotorOutputs(
        double position,
        double velocity,
        double voltage,
        double statorCurrent,
        double supplyCurrent,
        double temperatureCelsius
    ) {
        this.position = position;
        this.velocity = velocity;
        this.voltage = voltage;
        this.statorCurrent = statorCurrent;
        this.supplyCurrent = supplyCurrent;
        this.temperatureCelsius = temperatureCelsius;
    }

    public MotorOutputs() {
        this(0, 0, 0, 0, 0, 0);
    }
}