package frc.lib.sim;

import edu.wpi.first.wpilibj.Timer;

public abstract class SimObject {
    private double lastFPGASeconds;

    /**
     * Gets the position of the mechanism in it's units (meters for linear, radians for angular)
     */
    public abstract double getPosition();

    /**
     * Gets the velocity of the mechanism in its units (m/s for linear, rad/s for angular)
     */
    public abstract double getVelocity();

    public abstract double getStatorCurrent();

    public abstract double setVoltage(double volts);

    protected abstract void simulate(double deltaSeconds);

    public void update() {
        double currentFPGASeconds = Timer.getFPGATimestamp();
        simulate(currentFPGASeconds - lastFPGASeconds);
        lastFPGASeconds = currentFPGASeconds;
    }

    protected SimObject() {
        lastFPGASeconds = Timer.getFPGATimestamp();
    }
}
