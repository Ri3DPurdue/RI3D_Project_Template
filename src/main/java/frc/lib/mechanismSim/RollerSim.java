package frc.lib.mechanismSim;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation wrapper for flywheel-style rollers that focuses on angular velocity and current draw.
 * Position always returns zero because roller rotation typically is not tracked.
 */
public class RollerSim extends SimObject{
    private final FlywheelSim sim;

    /**
     * Create a roller simulation around a WPILib flywheel model.
     *
     * @param system underlying WPILib flywheel simulation
     */
    public RollerSim(FlywheelSim system) {
        sim = system;
    }

    @Override
    public Angle getPosition() {
        return BaseUnits.AngleUnit.zero(); // Rollers do not care about position and no simulated value exists so just return 0
    }

    @Override
    public AngularVelocity getVelocity() {
        return Units.RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
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
