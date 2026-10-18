package frc.lib.io.gyro;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class Pigeon2IO extends GyroIO {
    private Pigeon2 pigeon;

    public Pigeon2IO(int canID) {
        this.pigeon = new Pigeon2(canID);
    }

    public Pigeon2IO(int canID, String canbus) {
        this.pigeon = new Pigeon2(canID, canbus);
    }

    @Override
    public Rotation2d getYaw() {
        return this.pigeon.getRotation2d();
    }

    public void setYaw(Rotation2d yaw) {
        this.pigeon.setYaw(yaw.getDegrees());
    }

    @Override
    public Optional<Rotation2d> getPitch() {
        return Optional.of(new Rotation2d(this.pigeon.getPitch().getValue().in(Units.Radians)));
    }

    @Override
    public Optional<Rotation2d> getRoll() {
        return Optional.of(new Rotation2d(this.pigeon.getRoll().getValue().in(Units.Radians)));
    }
}
