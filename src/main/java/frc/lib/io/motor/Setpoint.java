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

    public Setpoint(Type outputType, double value) {
        this.outputType = outputType;
        this.value = value;
    }

    public static final Setpoint IDLE = new Setpoint(Type.Idle, 0);
}
