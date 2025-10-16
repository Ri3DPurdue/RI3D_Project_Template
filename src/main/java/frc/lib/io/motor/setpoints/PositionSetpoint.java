package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Angle;

public class PositionSetpoint extends BaseSetpoint<Angle> {
    public PositionSetpoint(Angle position) {
        super(position);
    }
}
