package frc.lib.io.motor;

import static frc.lib.io.motor.Setpoint.Type;

import edu.wpi.first.math.system.plant.DCMotor;

public abstract class MotorIO {
    private Setpoint currentSetpoint;
    private boolean enabled;
    private MotorOutputs[] outputs;

    protected MotorIO(int numFollowers) {
        currentSetpoint = new Setpoint(Type.Idle, 0);
        outputs = new MotorOutputs[numFollowers + 1];
        enabled = true;
    }

    public final void enable() {
        enabled = true;
        applySetpoint(currentSetpoint);
    }

    public final void disable() {
        enabled = false;
        setIdle();
    }

    public final void applySetpoint(Setpoint setpoint) {
        if (enabled) {
            currentSetpoint.set(setpoint);

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

    public final Setpoint getCurrentSetpoint() {
        return currentSetpoint.clone();
    }

    public final MotorOutputs[] getOutputs() {
        return outputs;
    }

    public void periodic() {
        updateOutputs(outputs);
    }

    protected abstract void updateOutputs(MotorOutputs[] outputs);

    protected abstract void setVoltage(double voltage);
    protected abstract void setCurrent(double current);

    protected abstract void setPosition(double position);
    protected abstract void setVelocity(double velocity);
    protected abstract void setProfiledPosition(double position);

    protected abstract void setPercentage(double percentage);
    protected abstract void setIdle();

    public abstract DCMotor getMotor();
}
