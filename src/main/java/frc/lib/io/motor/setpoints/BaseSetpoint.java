package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.*;

/**
 * Base class for a generic setpoint
 * @param <M> The type of measure of the setpoint
 */
public abstract class BaseSetpoint<M extends Measure<? extends Unit>> {
    M value;

    /**
     * Initializes this setpoint with a base value
     * @param value The base value of the setpoint
     */
    public BaseSetpoint(M value) {
        this.value = value;
    }

    /**
     * @return The current value of the setpoint
     */
    public M get() {
        return value;
    }

    /**
     * @return The name of the setpoint type, useful for logging the setpoint type
     */
    public final String getName() {
        return getClass().getSimpleName();
    }

    /**
     * @return Whether this is a position setpoint
     */
    public boolean isPositionSetpoint() {
        return this instanceof PositionSetpoint;
    }

    /**
     * @return Whether this is a velocity setpoint
     */
    public boolean isVelocitySetpoint() {
        return this instanceof VelocitySetpoint;
    }

    /**
     * @return Whether this is a voltage setpoint
     */
    public boolean isVoltageSetpoint() {
        return this instanceof VoltageSetpoint;
    }

    /**
     * @return Whether this is a current setpoint
     */
    public boolean isCurrentSetpoint() {
        return this instanceof CurrentSetpoint;
    }

    /**
     * @return Whether this is a idle setpoint
     */
    public boolean isIdleSetpoint() {
        return this instanceof IdleSetpoint;
    }
}