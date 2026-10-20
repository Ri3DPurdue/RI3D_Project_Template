package frc.lib.io.sensor.gyro;

import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public abstract class GyroIO implements Loggable {
    /*
     * Gets the yaw of the gyroscope with CCW positive
     */
    public abstract Angle getYaw();

    /*
     * Resets the yaw of the gyroscope
     */
    public abstract void setYaw(Angle yaw);

    /*
     * Sets the yaw of the gyroscope to zero
     */
    public void zero() {
        setYaw(Units.Degrees.zero());
    }

    /*
     * Gets the pitch of the gyroscope
     * Return empty for not supported
     */
    public Optional<Angle> getPitch() {
        return Optional.empty();
    }

    /*
     * Gets the roll of the gyroscope
     * Return empty for not supported
     */
    public Optional<Angle> getRoll() {
        return Optional.empty();
    }

    public void periodic() {}

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
