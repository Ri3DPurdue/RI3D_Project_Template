package frc.lib.io.motor;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorOutputs {
    public Angle position;
    public AngularVelocity velocity;

    public Voltage supplyVoltage;
    public Voltage statorVoltage;
    
    public Current statorCurrent;
    public Current supplyCurrent;
    public Temperature temperatureCelsius;

    public MotorOutputs() {
        this(BaseUnits.AngleUnit.zero(), 
        BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).zero(), 
        BaseUnits.VoltageUnit.zero(), 
        BaseUnits.VoltageUnit.zero(), 
        BaseUnits.CurrentUnit.zero(), 
        BaseUnits.CurrentUnit.zero(), 
        BaseUnits.TemperatureUnit.zero() 
        );
    }

    public MotorOutputs(
        Angle position,
        AngularVelocity velocity,
        Voltage supplyVoltage,
        Voltage statorVoltage,
        Current statorCurrent,
        Current supplyCurrent,
        Temperature temperatureCelsius
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