package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.AngularVelocity;

public class VelocitySetpoint extends BaseSetpoint<AngularVelocity> {
    public VelocitySetpoint(AngularVelocity velocity) {
        super(velocity);
    }
}
