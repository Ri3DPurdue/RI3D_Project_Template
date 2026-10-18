package frc.lib.io.sensor.encoder;


import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public abstract class EncoderIO implements Loggable {
    /*
     * Gets the offset position of the encoder
     * Should be CCW+ if inverted is false
     */
    public abstract Rotation2d getPosition();
    public abstract void setPosition(Rotation2d position);
    public abstract void setOffset(Rotation2d offset);
    public abstract void setInverted(boolean inverted);

    @Override
    public void log(String path) {
        Logger.log(path, "Position", getPosition());
    }
}