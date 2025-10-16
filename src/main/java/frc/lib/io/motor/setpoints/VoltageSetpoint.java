package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;

public class VoltageSetpoint extends BaseSetpoint<VoltageUnit, Voltage> {
    public VoltageSetpoint(Voltage voltage) {
        super(voltage);
    }
}
