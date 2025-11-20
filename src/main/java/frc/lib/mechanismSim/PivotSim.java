package frc.lib.mechanismSim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotSim extends SimObject {
    private final SingleJointedArmSim sim;

    public PivotSim(SingleJointedArmSim system) {
        sim = system;
    }

    @Override
    public Angle getPosition() {
        return Units.Radians.of(sim.getAngleRads());
    }

    @Override
    public AngularVelocity getVelocity() {
        return Units.RadiansPerSecond.of(sim.getVelocityRadPerSec());
    }

    @Override
    public Current getStatorCurrent() {
        return Units.Amps.of(sim.getCurrentDrawAmps());
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
