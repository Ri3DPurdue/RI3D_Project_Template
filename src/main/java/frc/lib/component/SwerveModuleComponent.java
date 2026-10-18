package frc.lib.component;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.io.motor.setpoints.VelocitySetpoint;
import frc.lib.io.motor.setpoints.VoltageSetpoint;
import frc.lib.io.sensor.encoder.EncoderIO;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;
import frc.lib.util.logging.Logger;

public class SwerveModuleComponent<D extends MotorIO, S extends MotorIO, E extends EncoderIO> implements Component {
    private final FlywheelMotorComponent<D> driveMotor;
    private final ServoMotorComponent<S> steerMotor;
    private final E encoder;
    private final DistanceAngleConverter conv;

    public SwerveModuleComponent(D driveMotor, AngularVelocity driveEpsilon, S steerMotor, Angle steerEpsilon, E driveEncoder, Distance wheelRadius) {
        this.driveMotor = new FlywheelMotorComponent<>(
            driveMotor,
            driveEpsilon
        );
        this.steerMotor = new ServoMotorComponent<>(
            steerMotor,
            steerEpsilon,
            driveEncoder.getPosition()
        );
        this.encoder = driveEncoder;
        this.conv = new DistanceAngleConverter(wheelRadius);
    }

    public SwerveModuleComponent(D driveMotor, S steerMotor, E driveEncoder, Distance wheelRadius) {
        this(
            driveMotor,
            Units.RPM.of(10),
            steerMotor,
            Units.Degrees.of(5),
            driveEncoder,
            wheelRadius
        );
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
        encoder.log(path, "Drive Encoder");
        steerMotor.log(path, "Steer Motor");

        double steerActual = steerMotor.getPosition().in(Units.Degrees) % 360.0;
        if (steerActual < 0) {
            steerActual += 360.0;
        }

        Logger.log(path, "Actual Angle", Rotation2d.fromDegrees(steerActual));
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
        return driveMotor.isEnabled() || steerMotor.isEnabled();
    }

    public FlywheelMotorComponent<D> getDriveComponent() {
        return driveMotor;
    }

    public ServoMotorComponent<S> getSteerComponent() {
        return steerMotor;
    }

    public E getEncoder() {
        return encoder;
    }

    public Distance getDriveDistance() {
        return conv.toDistance(getDriveComponent().getPosition());
    }

    public Angle getSteerAngle() {
        return getSteerComponent().getPosition();
    }

    public void resetToAbsolute() {
        this.steerMotor.resetPosition(this.encoder.getPosition());
    }

    /**
     * Applies a new control setpoint to the module
     * 
     * @param angle the angle to rotate the wheel at
     * @param vel the angular velocity to rotate the drive wheel at
     */
    public void applySetpoint(Rotation2d angle, LinearVelocity vel) {
        steerMotor.applySetpoint(new PositionSetpoint(Units.Radians.of(angle.getRadians())));
        driveMotor.applySetpoint(new VelocitySetpoint(conv.toAngle(vel)));
    }

    /**
     * Applies a new control setpoint to the module
     * 
     * @param angle the angle to rotate the wheel at
     * @param voltage the voltage to run the drive motor at
     */
    public void applySetpoint(Rotation2d angle, Voltage voltage) {
        steerMotor.applySetpoint(new PositionSetpoint(Units.Radians.of(angle.getRadians())));
        driveMotor.applySetpoint(new VoltageSetpoint(voltage));
    }

    /**
     * Creates a command that applies a setpoint once and ends immediately.
     * 
     * @param angle the angle to rotate the wheel at
     * @param vel the angular velocity to rotate the drive wheel at
     */
    public Command applySetpointCommand(Rotation2d angle, LinearVelocity vel) {
        return Commands.runOnce(() -> applySetpoint(angle, vel));
    }

    /**
     * Creates a command that applies a setpoint once and ends immediately.
     * 
     * @param angle the angle to rotate the wheel at
     * @param voltage the voltage to run the drive motor at
     */
    public Command applySetpointCommand(Rotation2d angle, Voltage voltage) {
        return Commands.runOnce(() -> applySetpoint(angle, voltage));
    }

    public static class SwerveModuleVelocitySetpoint {
        public Rotation2d angle;
        public LinearVelocity vel;

        public SwerveModuleVelocitySetpoint(Rotation2d angle, LinearVelocity vel) {
            this.angle = angle;
            this.vel = vel;
        }
    }
    
    public static class SwerveModuleVoltageSetpoint {
        public Rotation2d angle;
        public Voltage voltage;

        public SwerveModuleVoltageSetpoint(Rotation2d angle, Voltage voltage) {
            this.angle = angle;
            this.voltage = voltage;
        }
    }

    /**
     * Creates a command that continuously follows a dynamic setpoint.
     * 
     * @apiNote Unfortunately, Java generics don't let you override a method
     * with a generic of a different type (cannot have a(Supplier[A]) and
     * void a(Supplier[B]) (using square brackets because cannot use angle
     * brackets, apparently))
     * @param angle the angle to rotate the wheel at
     * @param vel the angular velocity to rotate the drive wheel at
     */
    public Command followVelocitySetpointCommand(Supplier<SwerveModuleVelocitySetpoint> supplier) {
        return Commands.run(() -> {
            SwerveModuleVelocitySetpoint s = supplier.get();
            applySetpoint(s.angle, s.vel);
        });
    }

    /**
     * Creates a command that continuously follows a dynamic setpoint.
     * 
     * @param angle the angle to rotate the wheel at
     * @param vel the angular velocity to rotate the drive wheel at
     */
    public Command followVoltageSetpointCommand(Supplier<SwerveModuleVoltageSetpoint> supplier) {
        return Commands.run(() -> {
            SwerveModuleVoltageSetpoint s = supplier.get();
            applySetpoint(s.angle, s.voltage);
        });
    }

    /**
     * Creates a command that enables the motor controllers.
     * 
     * @return a command that enables the motors once and ends immediately
     */
    public Command enableCommand() {
        return Commands.runOnce(() -> enable());
    }

    /**
     * Creates a command that disables the motor controllers.
     * 
     * @return a command that disables the motors once and ends immediately
     */
    public Command disableCommand() {
        return Commands.runOnce(() -> disable());
    }
}
