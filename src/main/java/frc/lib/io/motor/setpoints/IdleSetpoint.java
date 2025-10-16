package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;

public class IdleSetpoint extends BaseSetpoint<Dimensionless> {
    public IdleSetpoint() {
        super(Units.Value.of(0));
    }
}
