package frc.lib.component;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import frc.lib.io.sensor.gyro.GyroIO;

public class GyroIOComponent implements Component {
    private final GyroIO gyro;

    public GyroIOComponent(GyroIO gyro) {
        this.gyro = gyro;
    }

    @Override
    public void periodic() { 
        gyro.periodic();    
    }

    @Override
    public void log(String path) {
        gyro.log(path);
    }

    public Angle getYaw() {
        return gyro.getYaw();
    }

    public void setYaw(Angle yaw) {
        gyro.setYaw(yaw);
    }

    public Optional<Angle> getPitch() {
        return gyro.getPitch();
    }

    public Optional<Angle> getRoll() {
        return gyro.getRoll();
    }
}
