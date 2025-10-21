package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.*;

public abstract class BaseSetpoint<M extends Measure<? extends Unit>> {
    M value;

    public BaseSetpoint(M value) {
        this.value = value;
    }

    public M get() {
        return value;
    }

    public final String getName() {
        return getClass().getSimpleName();
    }

    public boolean isPositionSetpoint() {
        return this instanceof PositionSetpoint;
    }

    public boolean isVelocitySetpoint() {
        return this instanceof VelocitySetpoint;
    }

    public boolean isVoltageSetpoint() {
        return this instanceof VoltageSetpoint;
    }

    public boolean isCurrentSetpoint() {
        return this instanceof CurrentSetpoint;
    }

    public boolean isIdleSetpoint() {
        return this instanceof IdleSetpoint;
    }
}