package frc.lib.io.sensor.gyro;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public class Pigeon2IOSim extends Pigeon2IO {
    private final Supplier<Angle> yaw;
    public Pigeon2IOSim(int canID, Supplier<Angle> simulatedYaw) {
        this(canID, "", simulatedYaw);
    }

    public Pigeon2IOSim(int canID, String canbus, Supplier<Angle> simulatedYaw) {
        super(canID, canbus);
        yaw = simulatedYaw;
    }

    @Override
    public void periodic() {
        pigeon.getSimState().setRawYaw(yaw.get());
        super.periodic();
    }


    
}
