package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class ProfiledPositionSetpoint extends BaseSetpoint<AngleUnit, Angle> {
    public ProfiledPositionSetpoint(Angle position) {
        super(position);
    }
}
