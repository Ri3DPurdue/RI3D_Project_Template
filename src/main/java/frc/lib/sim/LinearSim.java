package frc.lib.sim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;

public class LinearSim extends SimObject {
    private final ElevatorSim sim;
    private final DistanceAngleConverter converter;

    public LinearSim(ElevatorSim system, DistanceAngleConverter distanceAngleConverter) {
        sim = system;
        converter = distanceAngleConverter;
    }
    
    @Override
    public Angle getPosition() {
        return converter.toAngle(Units.Meters.of(sim.getPositionMeters()));
    }

    @Override
    public AngularVelocity getVelocity() {
        return converter.toAngle(Units.MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
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
