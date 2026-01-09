package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Angle;

/**
 * A setpoint for position
 */
public class PositionSetpoint extends BaseSetpoint<Angle> {
    /**
     * Initializes a setpoint with a specified position
     * @param current The position to target
     */
    public PositionSetpoint(Angle position) {
        super(position);
    }
}
