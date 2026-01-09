package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;

/**
 * A setpoint for idling
 */
public class IdleSetpoint extends BaseSetpoint<Dimensionless> {
    /**
     * Initializes an idle setpoint
     */
    public IdleSetpoint() {
        super(Units.Value.of(0));
    }
}
