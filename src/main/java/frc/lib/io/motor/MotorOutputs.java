package frc.lib.io.motor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class MotorOutputs implements Loggable {
    public Angle position;
    public AngularVelocity velocity;
    public Voltage supplyVoltage;
    public Voltage statorVoltage;
    public Current statorCurrent;
    public Current supplyCurrent;
    public Temperature temperature;

    private AngleUnit loggedPositionUnit;
    private AngularVelocityUnit loggedVelocityUnit;
    private TemperatureUnit loggedTemperatureUnit;

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
        loggedPositionUnit = Radians;
        loggedVelocityUnit = RadiansPerSecond;
        loggedTemperatureUnit = Celsius;
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Position", position, loggedPositionUnit);
        Logger.log(path, "Velocity", velocity, loggedVelocityUnit);
        Logger.log(path, "Supply Voltage", supplyVoltage);
        Logger.log(path, "Stator Voltage", statorVoltage);
        Logger.log(path, "Stator Current", statorCurrent);
        Logger.log(path, "Supply Current", supplyCurrent);
        Logger.log(path, "Temperature", temperature, loggedTemperatureUnit);
    }

    public void overrideLoggedUnits(
        AngleUnit loggedPositionUnit,
        AngularVelocityUnit loggedVelocityUnit,
        TemperatureUnit loggedTemperatureUnit
    ) {
        this.loggedPositionUnit = loggedPositionUnit;
        this.loggedVelocityUnit = loggedVelocityUnit;
        this.loggedTemperatureUnit = loggedTemperatureUnit;
    }
}