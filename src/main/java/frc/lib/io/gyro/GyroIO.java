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
     * Gets the pitch of the gyroscope
     * Defaults to throwing an 
     * UnsupportedOperationException
     */
    public void getPitch() {
        throw new UnsupportedOperationException("getPitch is unsupported on the gyro");
    }

    /*
     * Sets the pitch of the gyroscope
     * Defaults to throwing an 
     * UnsupportedOperationException
     */
    public void setPitch() {
        throw new UnsupportedOperationException("setPitch is unsupported on the gyro");
    }

    /*
     * Sets the yaw of the gyroscope to zero
     */
    public void zero() {
        setYaw(new Rotation2d());
    }
}
