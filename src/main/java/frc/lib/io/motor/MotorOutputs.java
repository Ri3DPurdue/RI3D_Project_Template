package frc.lib.io.motor;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Util.logging.Loggable;
import frc.lib.Util.logging.Logger;

public class MotorOutputs implements Loggable {
    public Angle position;
    public AngularVelocity velocity;

    public Voltage supplyVoltage;
    public Voltage statorVoltage;
    
    public Current statorCurrent;
    public Current supplyCurrent;
    public Temperature temperature;

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
        Temperature temperature
    ) {
        this.position = position;
        this.velocity = velocity;
        this.supplyVoltage = supplyVoltage;
        this.statorVoltage = statorVoltage;
        this.statorCurrent = statorCurrent;
        this.supplyCurrent = supplyCurrent;
        this.temperature = temperature;
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Position", position);
        Logger.log(path, "Velocity", velocity);
        Logger.log(path, "Supply Voltage", supplyVoltage);
        Logger.log(path, "Stator Voltage", statorVoltage);
        Logger.log(path, "Stator Current", statorCurrent);
        Logger.log(path, "Supply Current", supplyCurrent);
        Logger.log(path, "Temperature", temperature);
    }
}