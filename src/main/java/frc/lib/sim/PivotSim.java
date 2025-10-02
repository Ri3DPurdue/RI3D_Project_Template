package frc.lib.sim;

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
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    protected void simulate(double deltaSeconds) {
        sim.update(deltaSeconds);
    }
    
}
