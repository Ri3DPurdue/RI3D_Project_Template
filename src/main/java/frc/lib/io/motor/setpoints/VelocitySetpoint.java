package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * A setpoint for velocity
 */
public class VelocitySetpoint extends BaseSetpoint<AngularVelocity> {
    /**
     * Initializes a setpoint with a specified velocity
     * @param current The velocity to target
     */
    public VelocitySetpoint(AngularVelocity velocity) {
        super(velocity);
    }
}
