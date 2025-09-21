package frc.lib.io.motor;

import static frc.lib.io.motor.Setpoint.Type;

public abstract class MotorIO {
    private Setpoint currentSetpoint;
    private boolean enabled;
    private MotorOutputs[] outputs;

    protected MotorIO(int numFollowers) {
        currentSetpoint = new Setpoint(Type.Idle, 0);
        outputs = new MotorOutputs[numFollowers + 1];
        for (int i = 0; i < numFollowers + 1; i++) {
            outputs[i] = new MotorOutputs();
        }

        enabled = true;
    }

    /**
     * Enables the motor, causing it to take the previously set setpoint
     * Note: A setpoint will still be applied, even if the motor is disabled,
     * it just won't take effect until the motor is re-enabled
     */
    public final void enable() {
        enabled = true;
        applySetpoint(currentSetpoint);
    }

    /**
     * Disabled the motor, causing it to idle until re-enabled
     * Note: Setpoints will still be applied, but it will only take effect once
     * the motor gets re-enabled
     */
    public final void disable() {
        enabled = false;
        setIdle();
    }

    public final boolean isEnabled() {
        return enabled;
    }

    /**
     * Applies the given setpoint to the motor
     * Note: Copies the value from the supplied setpoint so you can keep
     * ownership of the value supplied
     * 
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

    /**
     * Gets the outputs for the motor
     * @param outputs The array to write the outputs into
     */
    protected abstract void updateOutputs(MotorOutputs[] outputs);

    protected abstract void setVoltage(double voltage);
    protected abstract void setCurrent(double current);

    protected abstract void setPosition(double position);
    protected abstract void setVelocity(double velocity);
    protected abstract void setProfiledPosition(double position);

    protected abstract void setPercentage(double percentage);
    protected abstract void setIdle();
}
