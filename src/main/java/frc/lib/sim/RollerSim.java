package frc.lib.sim;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerSim extends SimObject{
    private final FlywheelSim sim;

    public RollerSim(FlywheelSim system) {
        sim = system;
    }

    @Override
    public double getPosition() {
        return Double.NaN; // Rollers do not care about position. (Replace with 0 if NaN causes issues)
    }

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRadPerSec();
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
