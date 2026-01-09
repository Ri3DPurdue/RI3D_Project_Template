package frc.lib.io.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Digital IO implementation for roboRIO Digital Input ports.
 * Provides debounced reading from a physical digital input channel
 * on the roboRIO's DIO ports.
 */
public class DigitalInIO extends DigitalIO {
    /** The WPILib DigitalInput object for reading the physical port */
    private final DigitalInput input;

    /**
     * Creates a new DigitalInIO on the specified DIO port with debouncing.
     * 
     * @param debounceSeconds The debounce time in seconds
     * @param port            The DIO port number on the roboRIO (0-9)
     */
    public DigitalInIO(double debounceSeconds, int port) {
        super(debounceSeconds);
        input = new DigitalInput(port);
    }

    /**
     * Gets the current value from the digital input port.
     * 
     * @return The state of the digital input (true = high, false = low)
     */
    @Override
    public boolean get() {
        return input.get();
    }

}
