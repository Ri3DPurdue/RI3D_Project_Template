package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Current;

/**
 * A setpoint for stator current
 */
public class CurrentSetpoint extends BaseSetpoint<Current> {
    /**
     * Initializes a setpoint with a specified current
     * @param current The current to target
     */
    public CurrentSetpoint(Current current) {
        super(current);
    }
}
