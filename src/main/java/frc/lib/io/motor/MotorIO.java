package frc.lib.io.motor;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import edu.wpi.first.units.measure.*;
import frc.lib.io.motor.setpoints.*;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Angle;

public abstract class MotorIO implements Loggable {
    private BaseSetpoint<?> currentSetpoint;
    private boolean enabled;
    private MotorOutputs[] outputs;
    private AngleUnit loggedPositionUnit;
    private AngularVelocityUnit loggedVelocityUnit;

    /**
     * Sets up the internal state for a MotorIO
     * @throws IllegalArgumentException If numFollowers is less than 0
     * @param numFollowers
     */
    protected MotorIO(int numFollowers) {
        if (numFollowers < 0) {
            throw new IllegalArgumentException("Number of followers must be non-negative");
        }

        currentSetpoint = new IdleSetpoint();
        outputs = new MotorOutputs[numFollowers + 1];
        for (int i = 0; i < numFollowers + 1; i++) {
            outputs[i] = new MotorOutputs();
        }
        enabled = true;
        loggedPositionUnit = Radian;
        loggedVelocityUnit = RadiansPerSecond;
    }

    /**
     * <p>
     * Enables the motor, causing it to take the previously set setpoint
     * </p>
     * <p>
     * Note: A setpoint will still be applied, even if the motor is disabled,
     * it just won't take effect until the motor is re-enabled
     * </p>
     */
    public final void enable() {
        enabled = true;
        applySetpoint(currentSetpoint);
    }

    /**
     * <p>
     * Disabled the motor, causing it to idle until re-enabled
     * </p>
     * <p>
     * Note: Setpoints will still be applied, but it will only take effect once
     * the motor gets re-enabled
     * </p>
     */
    public final void disable() {
        enabled = false;
        setIdle();
    }

    public final boolean isEnabled() {
        return enabled;
    }

    /**
     * <p>
     * Applies the given setpoint to the motor
     * </p>
     * <p>
     * Note: Copies the value from the supplied setpoint so you can keep
     * ownership of the value supplied
     * </p>
     * @param setpoint
     */
    public final void applySetpoint(BaseSetpoint<?> setpoint) {
        currentSetpoint = setpoint;

        if (enabled) {
            // Because profiled position setpoint is a subclass of
            // position setpoint, this check needs to be first.
            // It is just nicer to not have to explicitly allow both position
            // and profiled position if they both take an angle.
            // However, it does enforce some ordering on this side
            if (setpoint instanceof ProfiledPositionSetpoint p) {
                setProfiledPosition(p.get());
            } else if (setpoint instanceof PositionSetpoint p) {
                setPosition(p.get());
            } else if (setpoint instanceof VelocitySetpoint v) {
                setVelocity(v.get());
            } else if (setpoint instanceof VoltageSetpoint v) {
                setVoltage(v.get());
            } else if (setpoint instanceof CurrentSetpoint c) {
                setCurrent(c.get());
            } else if (setpoint instanceof IdleSetpoint) {
                setIdle();
            } else {
                throw new RuntimeException("Unknown setpoint type. Please use one of the given setpoint types in frc.lib.motors.setpoints");
            }
        }
    }

    /**
     * Gets the current setpoint
     * @implNote This does get updated even when the motor is disabled
     */
    public final BaseSetpoint<?> getCurrentSetpoint() {
        return currentSetpoint;
    }

    /**
     * Gets the outputs from all the motors
     * @implNote This returns the internal array, so be careful about modifying it
     */
    public final MotorOutputs[] getOutputs() {
        return outputs;
    }

    /**
     * Method to be run periodically for a motor
     * @implNote This is intended to be able to be overridden by subclasses
     */
    public void periodic() {
        updateOutputs(outputs);
    }

    @Override
    public void log(String path) {
        BaseSetpoint<?> setpoint = getCurrentSetpoint();
        if (setpoint instanceof PositionSetpoint p) {
            Logger.log(path, "Setpoint Value", p.get(), loggedPositionUnit);
        } else if (setpoint instanceof VelocitySetpoint v) {
            Logger.log(path, "Setpoint Value", v.get(), loggedVelocityUnit);
        } else {
            Logger.log(path, "Setpoint Value", setpoint.get());
        }
        Logger.log(path, "Setpoint Type", getCurrentSetpoint().getName());
        Logger.log(path, "Main", outputs[0]);
        Logger.log(path, "Followers", Arrays.copyOfRange(outputs, 1, outputs.length));
    }

    public void overrideLoggedUnits(
        AngleUnit loggedPositionUnit,
        AngularVelocityUnit loggedVelocityUnit,
        TemperatureUnit loggedTemperatureUnit
    ) {
        this.loggedPositionUnit = loggedPositionUnit;
        this.loggedVelocityUnit = loggedVelocityUnit;
        for (MotorOutputs output : outputs) {
            output.overrideLoggedUnits(loggedPositionUnit, loggedVelocityUnit, loggedTemperatureUnit);
        }
    }

    /**
     * Gets the outputs for the motor
     * @implNote The first element in the array is where the main motor output is to go
     * @param outputs The array to write the outputs into
     */
    protected abstract void updateOutputs(MotorOutputs[] outputs);

    protected abstract void setVoltage(Voltage voltage);
    protected abstract void setCurrent(Current current);

    protected abstract void setPosition(Angle angle);
    protected abstract void setVelocity(AngularVelocity velocity);
    protected abstract void setProfiledPosition(Angle position);

    protected abstract void setIdle();

    public abstract void useSoftLimits(boolean use);
    public abstract void resetPosition(Angle position);
}
