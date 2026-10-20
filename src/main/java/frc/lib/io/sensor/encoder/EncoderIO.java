package frc.lib.io.sensor.encoder;

import edu.wpi.first.units.measure.Angle;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public abstract class EncoderIO implements Loggable {
    /*
     * Gets the offset position of the encoder
     * Should be CCW+ if inverted is false
     */
    public abstract Angle getPosition();
    public abstract void setPosition(Angle position);
    public abstract void setOffset(Angle offset);
    public abstract void setInverted(boolean inverted);

    public void periodic() {}

    @Override
    public void log(String path) {
        Logger.log(path, "Position", getPosition());
    }
}