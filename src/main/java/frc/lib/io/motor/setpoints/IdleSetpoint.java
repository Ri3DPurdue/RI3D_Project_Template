package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;

public class IdleSetpoint extends BaseSetpoint<DimensionlessUnit, Dimensionless> {
    public IdleSetpoint() {
        super(Units.Value.of(0));
    }
}
