package frc.lib.sim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public abstract class SimObject {
    private Time lastFPGASeconds;

    /**
     * Get the position of the simulation. Updates when {@link #update()} is called.
     * 
     * @return Position of the simulation in radians if an angular system or meters if a linear system.
     */
    public abstract double getPosition();

    /**
     * Get the velocity of the simulation. Updates when {@link #update()} is called.
     * 
     * @return Velocity of the simulation in radians per second if an angular system or meters per second if 
     * a linear system. 
     */
    public abstract double getVelocity();

    /**
     * Get the current stator current of the simulation. Motor controller simulation stator currents should be 
     * used instead whenever possible. This should only be used if motor controller does not have a feature to 
     * get simulated stator current draw. Updates when {@link #update()} is called.
     * 
     * @return Stator current of the simulation in amps.
     */
    public abstract double getStatorCurrent();

    /**
     * Set the input voltage of the simulation. Should be updated based on simulated motor output before updating 
     * sim with {@link #update()}.
     * 
     * @param voltage Motor voltage.
     */
    public abstract void setVoltage(Voltage voltage);

    /**
     * Update the implementation specific internal simulation based on the time passed since the last update.
     * 
     * @param deltaSeconds The amount of time since the last simulation update.
     */
    protected abstract void simulate(Time deltaTime);

    /**
     * Update the simulation. Needs to be called periodically more frequent calls will result in higher sim resolution.
     */
    public void update() {
        Time currentFPGASeconds = Units.Seconds.of(Timer.getFPGATimestamp());
        simulate(currentFPGASeconds.minus(lastFPGASeconds));
        lastFPGASeconds = currentFPGASeconds;
    }

    /**
     * Construct a SimObject and set starting timestamp.
     */
    protected SimObject() {
        lastFPGASeconds = Units.Seconds.of(Timer.getFPGATimestamp());
    }
}
