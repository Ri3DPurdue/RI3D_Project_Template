package frc.lib.io.motor;

/**
 * Class that defines a setpoint to a MotorIO
 * Holds what kind of setpoint it is (Motor output type) and what value it takes
 * The interpretation of the value depends on the setpoint type
 */
public class Setpoint {
    public static enum Type {
        Voltage,
        Current,
        
        Position,
        Velocity,

        ProfiledPosition,

        Percentage,

        Idle;

        public boolean isPositionControl() {
            return switch (this) {
                case Position, ProfiledPosition -> true;
                default -> false;
            };
        }

        public boolean isVelocityControl() {
            return switch (this) {
                case Velocity -> true;
                default -> false;
            };
        }

        public boolean openLoop() {
            return switch (this) {
                case Voltage, Percentage, Idle, Current -> true;
                default -> false;
            };
        }
    }

    public Type outputType;
    public double value;

    /**
     * Private Setpoint constructor for easier internal construction
     */
    private Setpoint() {
        this.outputType = Type.Idle;
        this.value = 0;
    }

    public Setpoint(Type outputType, double value) {
        this.outputType = outputType;
        this.value = value;
    }

    /**
     * Copies the value from the given setpoint
     */
    public void set(Setpoint other) {
        this.outputType = other.outputType;
        this.value = other.value;
    }

    /**
     * Creates a new setpoint with the same values as the current one
     */
    public Setpoint clone() {
        Setpoint newSetpoint = new Setpoint();
        newSetpoint.set(this);

        return newSetpoint;
    }

    public static final Setpoint IDLE = new Setpoint();
}
