package frc.lib.io.sensor.gyro;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class Pigeon2IO extends GyroIO {
    protected Pigeon2 pigeon;

    public Pigeon2IO(int canID) {
        this.pigeon = new Pigeon2(canID);
    }

    public Pigeon2IO(int canID, String canbus) {
        this.pigeon = new Pigeon2(canID, canbus);
    }

    @Override
    public Angle getYaw() {
        return this.pigeon.getYaw().getValue();
    }

    @Override
    public void setYaw(Angle yaw) {
        this.pigeon.setYaw(yaw.in(Units.Degrees));
    }

    @Override
    public Optional<Angle> getPitch() {
        return Optional.of(this.pigeon.getPitch().getValue());
    }

    @Override
    public Optional<Angle> getRoll() {
        return Optional.of(this.pigeon.getRoll().getValue());
    }
}
