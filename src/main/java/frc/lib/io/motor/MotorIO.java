package frc.lib.io.motor;

import static frc.lib.io.motor.Setpoint.Type;

public abstract class MotorIO {
    private Setpoint currentSetpoint;
    private boolean enabled;

    protected MotorIO() {
        currentSetpoint = new Setpoint(Type.Idle, 0);
        enabled = true;
    }

    public void enable() {
        enabled = true;
        runSetpoint(currentSetpoint);
    }

    public void disable() {
        enabled = false;
        runSetpoint(Setpoint.IDLE);
    }

    public void applySetpoint(Setpoint setpoint) {
        if (enabled) {
            currentSetpoint.outputType = setpoint.outputType;
            currentSetpoint.value = setpoint.value;

            runSetpoint(currentSetpoint);
        }
    }

    public Setpoint getCurrentSetpoint() {
        Setpoint setpoint = new Setpoint(currentSetpoint.outputType, currentSetpoint.value);

        return setpoint;
    }

    protected abstract void runSetpoint(Setpoint setpoint);
}
