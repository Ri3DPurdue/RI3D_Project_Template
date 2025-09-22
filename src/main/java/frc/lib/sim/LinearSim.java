package frc.lib.sim;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class LinearSim extends SimObject {
    private final ElevatorSim sim;

    public LinearSim(ElevatorSim system) {
        sim = system;
    }
    
    @Override
    public double getPosition() {
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
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
