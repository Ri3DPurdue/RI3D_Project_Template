package frc.lib.component;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorOutputs;
import frc.lib.io.motor.setpoints.BaseSetpoint;
import frc.robot.subsystems.exampleIntake1.util.logging.Logger;

public class MotorComponent<M extends MotorIO> implements Component {
    private final M io;

    public MotorComponent(M motorIO) {
        io = motorIO;
    }

    @Override
    public void periodic() {
        io.periodic();
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Motor", io);
    }

    public void enable() {
        io.enable();
    }

    public void disable() {
        io.disable();
    }

    public boolean isEnabled() {
        return io.isEnabled();
    }

    /**
     * Gets the specific MotorIO for this component. Should only be used to access property specific to motor controller and not available through generic MotorIO.
     * 
     * @return
     */
    public M getMotorIO() {
        return io;
    }
    
    public MotorOutputs[] getMotorOutputs() {
        return io.getOutputs();
    }

    /**
     * Get the position of the main motor.
     * 
     * @return 
     */
    public Angle getPosition() {
        return getMotorOutputs()[0].position;
    }

    /**
     * Get the velocity of the main motor.
     * 
     * @return 
     */
    public AngularVelocity getVelocity() {
        return getMotorOutputs()[0].velocity;
    }

    /**
     * Get the stator voltage of the main motor.
     * 
     * @return 
     */
    public Voltage getStatorVoltage() {
        return getMotorOutputs()[0].statorVoltage;
    }

    /**
     * Get the voltage of the main motor.
     * 
     * @return 
     */
    public Voltage getSupplyVoltage() {
        return getMotorOutputs()[0].supplyVoltage;
    }

    /**
     * Get the stator current of the main motor.
     * 
     * @return 
     */
    public Current getStatorCurrent() {
        return getMotorOutputs()[0].statorCurrent;
    }

    /**
     * Get the supply current of the main motor.
     * 
     * @return 
     */
    public Current getSupplyCurrent() {
        return getMotorOutputs()[0].supplyCurrent;
    }

    /**
     * Get the temperature of the main motor.
     * 
     * @return 
     */
    public Temperature getTemperature() {
        return getMotorOutputs()[0].temperature;
    }

    /**
     * Get the setpoint of the MotorIO.
     * 
     * @return 
     */
    public BaseSetpoint<?> getSetpoint() {
        return io.getCurrentSetpoint();
    }

    public void useSoftLimits(boolean use) {
        io.useSoftLimits(use);
    }

    public void resetPosition(Angle position) {
        io.resetPosition(position);
    }

    public void applySetpoint(BaseSetpoint<?> setpoint) {
        io.applySetpoint(setpoint);
    }

    public Command applySetpointCommand(BaseSetpoint<?> setpoint) {
        return Commands.runOnce(() -> applySetpoint(setpoint));
    }

    public Command followSetpointCommand(Supplier<BaseSetpoint<?>> supplier) {
        return Commands.run(() -> applySetpoint(supplier.get()));
    }

    public Command enableCommand() {
        return Commands.runOnce(() -> enable());
    }

    public Command disableCommand() {
        return Commands.runOnce(() -> disable());
    }
}
