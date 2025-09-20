package frc.lib.io.motor;

import static frc.lib.io.motor.Setpoint.Type;

public abstract class MotorIO {
    private Setpoint currentSetpoint;
    private boolean enabled;
    private MotorOutputs outputs;

    protected MotorIO() {
        currentSetpoint = new Setpoint(Type.Idle, 0);
        outputs = new MotorOutputs();
        enabled = true;
    }

    public final void enable() {
        enabled = true;
        runSetpoint(currentSetpoint);
    }

    public final void disable() {
        enabled = false;
        runSetpoint(Setpoint.IDLE);
    }

    public final void applySetpoint(Setpoint setpoint) {
        if (enabled) {
            currentSetpoint.set(setpoint);

            runSetpoint(currentSetpoint);
        }
    }

    public final Setpoint getCurrentSetpoint() {
        return currentSetpoint.clone();
    }

    public void periodic() {
        updateOutputs(outputs);
    }

    protected abstract void runSetpoint(Setpoint setpoint);
    protected abstract void updateOutputs(MotorOutputs outputs);
}
