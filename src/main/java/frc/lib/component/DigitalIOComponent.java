package frc.lib.component;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Util.logging.Logger;
import frc.lib.io.sensor.DigitalIO;

/**
 * A component wrapper for digital I/O sensors.
 * 
 * <p>
 * This class provides a component-based interface for digital input sensors,
 * offering both raw and debounced readings, as well as command-based state
 * waiting.
 */
public class DigitalIOComponent implements Component {
    private final DigitalIO io;

    /**
     * Constructs a new DigitalIOComponent.
     * 
     * @param digitalIO the digital I/O interface to wrap
     */
    public DigitalIOComponent(DigitalIO digitalIO) {
        io = digitalIO;
    }

    /**
     * Updates the digital I/O sensor state.
     * 
     * <p>
     * This method should be called periodically to refresh sensor readings.
     */
    @Override
    public void periodic() {
        io.update();
    }

    /**
     * Logs the digital I/O state to the specified path.
     * 
     * @param path the logging path for this component's data
     */
    @Override
    public void log(String path) {
        Logger.log(path, io);
    }

    /**
     * Gets the current raw state of the digital input.
     * 
     * @return the current state of the digital input (true = high, false = low)
     */
    public boolean get() {
        return io.get();
    }

    /**
     * Gets the debounced state of the digital input.
     * 
     * <p>
     * Debouncing filters out rapid state changes to provide a stable reading.
     * 
     * @return the debounced state of the digital input (true = high, false = low)
     */
    public boolean getDebounced() {
        return io.getDebounced();
    }

    /**
     * Creates a command that waits until the digital input reaches the specified
     * state.
     * <p>
     * Reccomended to use in a sequential command group or .andThen() command.
     * 
     * <p>
     * Uses the raw (non-debounced) sensor reading.
     * 
     * @param state the target state to wait for (true = high, false = low)
     * @return a command that finishes when the sensor reaches the specified state
     */
    public Command stateWait(boolean state) {
        return Commands.waitUntil(() -> state == get());
    }

    /**
     * Creates a command that waits until the debounced digital input reaches the
     * specified state.
     * <p>
     * Reccomended to use in a sequential command group or .andThen() command.
     * 
     * <p>
     * Uses the debounced sensor reading, which is more stable and resistant to
     * noise.
     * 
     * @param state the target state to wait for (true = high, false = low)
     * @return a command that finishes when the debounced sensor reaches the
     *         specified state
     */
    public Command stateWaitDebounced(boolean state) {
        return Commands.waitUntil(() -> state == getDebounced());
    }
}
