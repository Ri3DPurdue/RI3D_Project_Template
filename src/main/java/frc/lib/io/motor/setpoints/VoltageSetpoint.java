package frc.lib.io.motor.setpoints;

import edu.wpi.first.units.measure.Voltage;

/**
 * A setpoint for stator voltage
 */
public class VoltageSetpoint extends BaseSetpoint<Voltage> {
    /**
     * Initializes a setpoint with a specified voltage
     * @param current The voltage to target
     */
    public VoltageSetpoint(Voltage voltage) {
        super(voltage);
    }
}
