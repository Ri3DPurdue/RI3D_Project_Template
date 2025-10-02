package frc.lib.io.motor;

public class BaseConfig {
    public static class PIDParams {
        public double p = 0.0;
        public double i = 0.0;
        public double d = 0.0;
        public double f = 0.0;
        public double g = 0.0;
    }

    public static class MotorInfo {
        public int canID = -1;
        public boolean inverted = false;
    }

    public static class ProfileParams {
        public double maxAcceleration = 0.0;
        public double maxVelocity = 0.0;
    }

    public PIDParams pid = new PIDParams();
    public ProfileParams profile = new ProfileParams();

    public MotorInfo main = new MotorInfo();
    public MotorInfo[] followers = new MotorInfo[0];

    /*
     * Ratio from physical motor rotor revolutions to 1 mechanism unit
     * What this unit depends on the mechanism (meters for linear, radians
     * for rotational)
     * This is a positional ratio, and the velocity ratio will be mechanism
     * units per second. That ratio should be automatically derived from this
     * number
     */
    public double mechanismRatio = 1.0;
    public int currentLimit = 40;
}
