package frc.lib.io.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

/**
 * Abstract base class for digital input/output sensors with debouncing.
 * Provides filtering of noisy digital signals through debouncing logic
 * and logging capabilities for both raw and debounced values.
 */
public abstract class DigitalIO implements Loggable {
    /**
     * Data container for digital input/output sensor readings.
     * Stores both raw and debounced sensor values.
     */
    public static class DigitalIOOutputs implements Loggable {
        /** The raw, unfiltered value from the digital sensor */
        public boolean raw = false;

        /** The debounced (filtered) value from the digital sensor */
        public boolean debounced = false;

        /**
         * Logs the digital sensor outputs to the specified path.
         * 
         * @param path The logging path for this data
         */
        @Override
        public void log(String path) {
            Logger.log(path, "Raw Ouput", raw);
            Logger.log(path, "Debounced Output", debounced);
        }
    }

    /** Debouncer filter for smoothing noisy digital signals */
    private final Debouncer debouncer;

    /** Container for sensor output values */
    private final DigitalIOOutputs outputs;

    /**
     * Creates a new DigitalIO sensor with debouncing.
     * 
     * @param debounceSeconds The debounce time in seconds (applies to both rising
     *                        and falling edges)
     */
    public DigitalIO(double debounceSeconds) {
        debouncer = new Debouncer(debounceSeconds, DebounceType.kBoth);
        outputs = new DigitalIOOutputs();
    }

    /**
     * Gets the raw digital sensor value.
     * This method must be implemented by subclasses to provide the actual sensor
     * reading.
     * 
     * @return The current raw state of the digital sensor
     */
    public abstract boolean get();

    /**
     * Updates the sensor readings and applies debouncing.
     * Should be called periodically to refresh both raw and debounced values.
     */
    public void update() {
        outputs.raw = get();
        outputs.debounced = debouncer.calculate(outputs.raw);
    }

    /**
     * Gets the debounced (filtered) digital sensor value.
     * This value has been processed to eliminate noise and transient changes.
     * 
     * @return The debounced state of the digital sensor
     */
    public boolean getDebounced() {
        return outputs.debounced;
    }

    /**
     * Logs all sensor data to the specified path.
     * 
     * @param path The logging path for this sensor
     */
    @Override
    public void log(String path) {
        Logger.log(path, outputs);
    }
}
