package frc.lib.io.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

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
}
