package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Current;

public class CurrentSetpoint extends BaseSetpoint<CurrentUnit, Current> {
    public CurrentSetpoint(Current current) {
        super(current);
    }
}
