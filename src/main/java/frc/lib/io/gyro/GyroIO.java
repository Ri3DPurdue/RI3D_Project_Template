package frc.lib.io.gyro;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public abstract class GyroIO implements Loggable {
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
     * Return empty for not supported
     */
    public Optional<Rotation2d> getPitch() {
        return Optional.empty();
    }

    /*
     * Gets the roll of the gyroscope
     * Return empty for not supported
     */
    public Optional<Rotation2d> getRoll() {
        return Optional.empty();
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Yaw", getYaw());

        // No gyro hotswapping.
        // If you wish to hotswap your gyro, then create a
        // PR or just update it yourself

        {
            var pitch = getPitch();

            if (pitch.isEmpty()) {
                Logger.log(path, "Pitch", "Unsupported");
            } else {
                Logger.log("Pitch", pitch.get());
            }
        }

        {
            var roll = getPitch();

            if (roll.isEmpty()) {
                Logger.log(path, "Roll", "Unsupported");
            } else {
                Logger.log("Roll", roll.get());
            }
        }
    }
}
