package frc.lib.io.motor;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorOutputs {
    public double position;
    public double velocity;

    public double supplyVoltage;
    public double statorVoltage;
    
    public double statorCurrent;
    public double supplyCurrent;
    public double temperatureCelsius;

    public MotorOutputs() {
        this(0, 0, 0, 0, 0, 0, 0);
    }

    public MotorOutputs(
        double position,
        double velocity,
        double supplyVoltage,
        double statorVoltage,
        double statorCurrent,
        double supplyCurrent,
        double temperatureCelsius
    ) {
        this.position = position;
        this.velocity = velocity;
        this.supplyVoltage = supplyVoltage;
        this.statorVoltage = statorVoltage;
        this.statorCurrent = statorCurrent;
        this.supplyCurrent = supplyCurrent;
        this.temperatureCelsius = temperatureCelsius;
    }
}