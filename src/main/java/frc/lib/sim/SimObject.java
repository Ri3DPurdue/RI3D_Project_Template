package frc.lib.sim;

import edu.wpi.first.wpilibj.Timer;

public abstract class SimObject {
    private double lastFPGASeconds;

    public abstract double getPosition();

    public abstract double getVelocity();

    public abstract double getStatorCurrent();

    public abstract void setVoltage(double volts);

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
