package frc.lib.io.motor;

import edu.wpi.first.units.measure.Angle;
import frc.lib.Util.logging.Loggable;

public abstract class MotorIO implements Loggable {
    private Setpoint currentSetpoint;
    private boolean enabled;
    private MotorOutputs[] outputs;

    /**
     * Sets up the internal state for a MotorIO
     * @throws IllegalArgumentException If numFollowers is less than 0
     * @param numFollowers
     */
    protected MotorIO(int numFollowers) {
        if (numFollowers < 0) {
            throw new IllegalArgumentException("Number of followers must be non-negative");
        }

        currentSetpoint = Setpoint.idleSetpoint();
        outputs = new MotorOutputs[numFollowers + 1];
        for (int i = 0; i < numFollowers + 1; i++) {
            outputs[i] = new MotorOutputs();
        }

        enabled = true;
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
    public final void applySetpoint(Setpoint setpoint) {
        currentSetpoint.set(setpoint);

        if (enabled) {
            switch (setpoint.outputType) {
                case Voltage:
                    setVoltage(setpoint.value);
                    break;

                case Current:
                    setCurrent(setpoint.value);
                    break;
                
                case Position:
                    setPosition(setpoint.value);
                    break;

                case Velocity:
                    setVelocity(setpoint.value);
                    break;

                case ProfiledPosition:
                    setProfiledPosition(setpoint.value);
                    break;

                case Percentage:
                    setPercentage(setpoint.value);
                    break;

                case Idle:
                    setIdle();
                    break;
            }
        }
    }

    /**
     * Gets the current setpoint
     * @implNote This does get updated even when the motor is disabled
     * @implNote This returns a copy of the current setpoint, so feel free to modify the data
     */
    public final Setpoint getCurrentSetpoint() {
        return currentSetpoint.clone();
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
        //TODO logging
        // Logger.recordOutput(path + "/Setpoint Base Units Value", getCurrentSetpoint().value); // TODO make log in the same place as MotorOutputs
        // Logger.recordOutput(path + "/Setpoint Output Type", getCurrentSetpoint().outputType); // TODO make log in the same place as MotorOutputs

        // Logger.processInputs(path, outputs[0]);

        // for (int i = 1; i < outputs.length; i++) {
        //     Logger.processInputs(path + "/Followers/" + i, outputs[i]);
        // }
    }

    /**
     * Gets the outputs for the motor
     * @implNote The first element in the array is where the main motor output is to go
     * @param outputs The array to write the outputs into
     */
    protected abstract void updateOutputs(MotorOutputs[] outputs);

    protected abstract void setVoltage(double volts);
    protected abstract void setCurrent(double amps);

    protected abstract void setPosition(double rads);
    protected abstract void setVelocity(double radsPerSecond);
    protected abstract void setProfiledPosition(double rads);

    protected abstract void setPercentage(double percentage);
    protected abstract void setIdle();
    public abstract void useSoftLimits(boolean use);
    public abstract void resetPosition(Angle position);
}
