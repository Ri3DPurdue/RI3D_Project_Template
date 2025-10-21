package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Voltage;

public class VoltageSetpoint extends BaseSetpoint<Voltage> {
    public VoltageSetpoint(Voltage voltage) {
        super(voltage);
    }
}
