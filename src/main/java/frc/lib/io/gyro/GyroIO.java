package frc.lib.io.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.logging.Logger;

public abstract class GyroIO extends Logger {
    /*
     * Gets the yaw of the gyroscope with CCW positive
     */
    public abstract Rotation2d getYaw();

    /*
     * Resets the yaw of the gyroscope
     */
    public abstract void setYaw(Rotation2d yaw);

    /*
     * Sets the yaw of the gyroscope to zero
     */
    public void zero() {
        setYaw(new Rotation2d());
    }

    /*
     * Gets the pitch of the gyroscope
     * Defaults to throwing an 
     * UnsupportedOperationException
     */
    public Rotation2d getPitch() {
        throw new UnsupportedOperationException("getPitch is unsupported on the gyro");
    }

    /*
     * Gets the roll of the gyroscope
     * Defaults to throwing an 
     * UnsupportedOperationException
     */
    public Rotation2d getRoll() {
        throw new UnsupportedOperationException("getRoll is unsupported on the gyro");
    }
}
