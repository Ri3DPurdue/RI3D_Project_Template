package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Angle;

/**
 * A setpoint for position achieved via a motion profile
 */
public class ProfiledPositionSetpoint extends PositionSetpoint {
    /**
     * Initializes a setpoint with a specified position
     * @param current The position to target
     */
    public ProfiledPositionSetpoint(Angle position) {
        super(position);
    }
}
