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
import frc.lib.util.logging.Logger;

/**
 * A component that wraps motor controller functionality.
 * 
 * <p>
 * This class provides a unified interface for controlling motors, reading
 * sensor data,
 * and managing motor setpoints. It supports various control modes through
 * setpoints and
 * provides convenient command factories for common motor operations.
 * 
 * <p>
 * The component handles periodic updates of motor I/O, logging of motor
 * telemetry,
 * and provides access to motor position, velocity, current, voltage, and
 * temperature.
 * 
 * @param <M> the type of MotorIO implementation used by this component
 * @see MotorIO
 * @see ServoMotorComponent
 * @see FlywheelMotorComponent
 */
public class MotorComponent<M extends MotorIO> implements Component {
    private final M io;

    /**
     * Constructs a new MotorComponent with the specified motor I/O interface.
     * 
     * @param motorIO the motor I/O implementation to control
     */
    public MotorComponent(M motorIO) {
        io = motorIO;
    }

    /**
     * Updates the motor I/O state.
     * 
     * <p>
     * This method should be called periodically to refresh motor sensor readings
     * and update control outputs.
     */
    @Override
    public void periodic() {
        io.periodic();
    }

    /**
     * Logs motor telemetry data to the specified path.
     * 
     * @param path the logging path for this component's data
     */
    @Override
    public void log(String path) {
        Logger.log(path, "Motor", io);
    }

    /**
     * Enables the motor controller.
     * 
     * <p>
     * When enabled, the motor will apply the current setpoint.
     */
    public void enable() {
        io.enable();
    }

    /**
     * Disables the motor controller.
     * 
     * <p>
     * When disabled, the motor will stop applying power regardless of setpoint.
     */
    public void disable() {
        io.disable();
    }

    /**
     * Checks if the motor controller is currently enabled.
     * 
     * @return true if the motor is enabled, false otherwise
     */
    public boolean isEnabled() {
        return io.isEnabled();
    }

    /**
     * Gets the specific MotorIO object for this component.
     * 
     * <p>
     * This method provides direct access to the underlying motor I/O interface.
     * It should only be used to access properties specific to the motor controller
     * implementation that are not available through the generic MotorIO interface.
     * 
     * @return the motor I/O implementation instance
     */
    public M getMotorIO() {
        return io;
    }

    /**
     * Gets all motor outputs including sensor readings and status information.
     * 
     * @return an array of MotorOutputs containing telemetry for all motors
     */
    public MotorOutputs[] getMotorOutputs() {
        return io.getOutputs();
    }

    /**
     * Gets the current position of the main motor.
     * 
     * @return the motor position as an Angle measurement
     */
    public Angle getPosition() {
        return getMotorOutputs()[0].position;
    }

    /**
     * Gets the current velocity of the main motor.
     * 
     * @return the motor velocity as an AngularVelocity measurement
     */
    public AngularVelocity getVelocity() {
        return getMotorOutputs()[0].velocity;
    }

    /**
     * Gets the stator voltage of the main motor.
     * 
     * <p>
     * The stator voltage is the voltage applied to the motor windings.
     * 
     * @return the stator voltage as a Voltage measurement
     */
    public Voltage getStatorVoltage() {
        return getMotorOutputs()[0].statorVoltage;
    }

    /**
     * Gets the supply voltage of the main motor.
     * 
     * <p>
     * The supply voltage is the voltage from the power source to the motor
     * controller.
     * 
     * @return the supply voltage as a Voltage measurement
     */
    public Voltage getSupplyVoltage() {
        return getMotorOutputs()[0].supplyVoltage;
    }

    /**
     * Gets the stator current of the main motor.
     * 
     * <p>
     * The stator current is the current flowing through the motor windings.
     * 
     * @return the stator current as a Current measurement
     */
    public Current getStatorCurrent() {
        return getMotorOutputs()[0].statorCurrent;
    }

    /**
     * Gets the supply current of the main motor.
     * 
     * <p>
     * The supply current is the current drawn from the power source.
     * 
     * @return the supply current as a Current measurement
     */
    public Current getSupplyCurrent() {
        return getMotorOutputs()[0].supplyCurrent;
    }

    /**
     * Gets the temperature of the main motor.
     * 
     * @return the motor temperature as a Temperature measurement
     */
    public Temperature getTemperature() {
        return getMotorOutputs()[0].temperature;
    }

    /**
     * Gets the current setpoint being applied to the motor.
     * 
     * @return the current setpoint, which may be position, velocity, voltage, etc.
     */
    public BaseSetpoint<?> getSetpoint() {
        return io.getCurrentSetpoint();
    }

    /**
     * Enables or disables soft limits for the motor.
     * 
     * <p>
     * Soft limits prevent the motor from exceeding configured position boundaries
     * in software, providing protection without physical limit switches.
     * 
     * @param use true to enable soft limits, false to disable them
     */
    public void useSoftLimits(boolean use) {
        io.useSoftLimits(use);
    }

    /**
     * Resets the motor's position encoder to a specified value.
     * 
     * <p>
     * This is typically used during homing or calibration procedures.
     * 
     * @param position the new position value to set
     */
    public void resetPosition(Angle position) {
        io.resetPosition(position);
    }

    /**
     * Applies a new control setpoint to the motor.
     * 
     * <p>
     * The setpoint determines the motor's control mode and target value.
     * Supported setpoint types include position, velocity, voltage, and current.
     * 
     * @param setpoint the setpoint to apply
     */
    public void applySetpoint(BaseSetpoint<?> setpoint) {
        io.applySetpoint(setpoint);
    }

    /**
     * Creates a command that applies a setpoint once and ends immediately.
     * 
     * @param setpoint the setpoint to apply
     * @return a command that applies the setpoint once
     */
    public Command applySetpointCommand(BaseSetpoint<?> setpoint) {
        return Commands.runOnce(() -> applySetpoint(setpoint));
    }

    /**
     * Creates a command that continuously follows a dynamic setpoint.
     * 
     * <p>
     * The supplier is called repeatedly to get updated setpoint values,
     * allowing for real-time setpoint adjustments during command execution.
     * 
     * @param supplier a supplier that provides the setpoint to follow
     * @return a command that continuously applies setpoints from the supplier
     */
    public Command followSetpointCommand(Supplier<BaseSetpoint<?>> supplier) {
        return Commands.run(() -> applySetpoint(supplier.get()));
    }

    /**
     * Creates a command that enables the motor controller.
     * 
     * @return a command that enables the motor once and ends immediately
     */
    public Command enableCommand() {
        return Commands.runOnce(() -> enable());
    }

    /**
     * Creates a command that disables the motor controller.
     * 
     * @return a command that disables the motor once and ends immediately
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> disable());
    }
}
