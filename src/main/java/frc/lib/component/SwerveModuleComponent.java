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

public class SwerveModuleComponent<D extends MotorIO, S extends MotorIO> implements Component {
    public class SwerveModuleSetpoint {
        public BaseSetpoint<?> steerSetpoint;
        public BaseSetpoint<?> driveSetpoint;

        public SwerveModuleSetpoint(BaseSetpoint<?> steerSetpoint, BaseSetpoint<?> driveSetpoint) {
            this.steerSetpoint = steerSetpoint;
            this.driveSetpoint = driveSetpoint;
        }
    }

    private D driveMotor;
    private S steerMotor;

    /**
     * Updates the motor I/O state.
     * 
     * <p>
     * This method should be called periodically to refresh motor sensor readings
     * and update control outputs.
     */
    @Override
    public void periodic() {
        driveMotor.periodic();
        steerMotor.periodic();
    }

    /**
     * Logs motor telemetry data to the specified path.
     * 
     * @param path the logging path for this component's data
     */
    @Override
    public void log(String path) {
        driveMotor.log(path, "Drive Motor");
        steerMotor.log(path, "Steer Motor");
    }

    /**
     * Enables both motor controllers.
     * 
     * <p>
     * When enabled, the motor will apply the current setpoint.
     */
    public void enable() {
        driveMotor.enable();
        steerMotor.enable();
    }

    /**
     * Disables both motor controllers.
     * 
     * <p>
     * When disabled, the motor will stop applying power regardless of setpoint.
     */
    public void disable() {
        driveMotor.disable();
        steerMotor.disable();
    }

    /**
     * Checks if both motor controllers are currently enabled.
     * 
     * @return true if the motor is enabled, false otherwise
     */
    public boolean isEnabled() {
        return driveMotor.isEnabled() && steerMotor.isEnabled();
    }

    public D getDriveMotor() {
        return driveMotor;
    }

    public S getSteerMotor() {
        return steerMotor;
    }

    /**
     * Gets all steer motor outputs including sensor readings and status information.
     * 
     * @return an array of MotorOutputs containing telemetry for all motors
     */
    public MotorOutputs[] getSteerOutputs() {
        return steerMotor.getOutputs();
    }


    /**
     * Gets all drive motor outputs including sensor readings and status information.
     * 
     * @return an array of MotorOutputs containing telemetry for all motors
     */
    public MotorOutputs[] getDriveOutputs() {
        return driveMotor.getOutputs();
    }

    /**
     * Gets the current position of the main motor.
     * 
     * @return the motor position as an Angle measurement
     */
    public Angle getSteerAngle() {
        return getSteerOutputs()[0].position;
    }

    /**
     * Gets the current velocity of the main motor.
     * 
     * @return the motor velocity as an AngularVelocity measurement
     */
    public AngularVelocity getDriveSpeed() {
        return getDriveOutputs()[0].velocity;
    }

    /**
     * Gets the stator voltage of the steer motor.
     * 
     * <p>
     * The stator voltage is the voltage applied to the motor windings.
     * 
     * @return the stator voltage as a Voltage measurement
     */
    public Voltage getSteerStatorVoltage() {
        return getSteerOutputs()[0].statorVoltage;
    }

    /**
     * Gets the stator voltage of the drive motor.
     * 
     * <p>
     * The stator voltage is the voltage applied to the motor windings.
     * 
     * @return the stator voltage as a Voltage measurement
     */
    public Voltage getDriveStatorVoltage() {
        return getDriveOutputs()[0].statorVoltage;
    }

    /**
     * Gets the supply voltage of the steer motor.
     * 
     * <p>
     * The supply voltage is the voltage from the power source to the motor
     * controller.
     * 
     * @return the supply voltage as a Voltage measurement
     */
    public Voltage getSteerSupplyVoltage() {
        return getSteerOutputs()[0].supplyVoltage;
    }

    /**
     * Gets the supply voltage of the drive motor.
     * 
     * <p>
     * The supply voltage is the voltage from the power source to the motor
     * controller.
     * 
     * @return the supply voltage as a Voltage measurement
     */
    public Voltage getDriveSupplyVoltage() {
        return getDriveOutputs()[0].supplyVoltage;
    }

    /**
     * Gets the stator current of the steer motor.
     * 
     * <p>
     * The stator current is the current flowing through the motor windings.
     * 
     * @return the stator current as a Current measurement
     */
    public Current getSteerStatorCurrent() {
        return getSteerOutputs()[0].statorCurrent;
    }

    /**
     * Gets the stator current of the drive motor.
     * 
     * <p>
     * The stator current is the current flowing through the motor windings.
     * 
     * @return the stator current as a Current measurement
     */
    public Current getDriveStatorCurrent() {
        return getDriveOutputs()[0].statorCurrent;
    }

    /**
     * Gets the supply current of the supply motor.
     * 
     * <p>
     * The supply current is the current drawn from the power source.
     * 
     * @return the supply current as a Current measurement
     */
    public Current getSteerSupplyCurrent() {
        return getSteerOutputs()[0].supplyCurrent;
    }

    /**
     * Gets the supply current of the drive motor.
     * 
     * <p>
     * The supply current is the current drawn from the power source.
     * 
     * @return the supply current as a Current measurement
     */
    public Current getDriveSupplyCurrent() {
        return getDriveOutputs()[0].supplyCurrent;
    }


    /**
     * Gets the temperature of the steer motor.
     * 
     * @return the motor temperature as a Temperature measurement
     */
    public Temperature getSteerTemperature() {
        return getSteerOutputs()[0].temperature;
    }

    /**
     * Gets the temperature of the drive motor.
     * 
     * @return the motor temperature as a Temperature measurement
     */
    public Temperature getDriveTemperature() {
        return getDriveOutputs()[0].temperature;
    }

    /**
     * Gets the current setpoint being applied to the motor.
     * 
     * @return the current setpoint, which may be position, velocity, voltage, etc.
     */
    public SwerveModuleSetpoint getSetpoint() {
        return new SwerveModuleSetpoint(steerMotor.getCurrentSetpoint(), driveMotor.getCurrentSetpoint());
    }

    /**
     * Resets the steer motor's position encoder to a specified value.
     * 
     * <p>
     * This is typically used during homing or calibration procedures.
     * 
     * @param position the new position value to set
     */
    public void resetSteeringAngle(Angle position) {
        steerMotor.resetPosition(position);
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
    public void applySetpoint(SwerveModuleSetpoint setpoint) {
        steerMotor.applySetpoint(setpoint.steerSetpoint);
        driveMotor.applySetpoint(setpoint.driveSetpoint);
    }

    /**
     * Creates a command that applies a setpoint once and ends immediately.
     * 
     * @param setpoint the setpoint to apply
     * @return a command that applies the setpoint once
     */
    public Command applySetpointCommand(SwerveModuleSetpoint setpoint) {
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
    public Command followSetpointCommand(Supplier<SwerveModuleSetpoint> supplier) {
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
