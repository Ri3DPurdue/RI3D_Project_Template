package frc.lib.io.motor;

public class Setpoint {
    public static enum Type {
        Voltage,
        Amperage,
        
        Position,
        Velocity,

        ProfiledPosition,

        Percentage,

        Idle
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

    public void set(Setpoint other) {
        this.outputType = other.outputType;
        this.value = other.value;
    }

    public Setpoint clone() {
        Setpoint newSetpoint = new Setpoint();
        newSetpoint.set(this);

        return newSetpoint;
    }

    public static final Setpoint IDLE = new Setpoint();
}
