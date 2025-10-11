package frc.lib.sim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim extends SimObject {
    private final SingleJointedArmSim sim;

    public PivotSim(SingleJointedArmSim system) {
        sim = system;
    }

    @Override
    public double getPosition() {
        return sim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getStatorCurrent() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        sim.setInputVoltage(voltage.in(Units.Volts));
    }

    @Override
    protected void simulate(Time deltaTime) {
        sim.update(deltaTime.in(Units.Seconds));
    }
    
}
