package frc.lib.io.sensor;

import java.util.function.BooleanSupplier;

/**
 * Digital IO implementation that wraps a BooleanSupplier.
 * Provides debouncing functionality for any boolean supplier function,
 * allowing flexible integration with various digital signal sources.
 */
public class DigitalBooleanSupplierIO extends DigitalIO {
    /** The boolean supplier providing the digital signal */
    private final BooleanSupplier input;

    /**
     * Creates a new DigitalBooleanSupplierIO with debouncing.
     * 
     * @param debounceSeconds The debounce time in seconds
     * @param supplier        The BooleanSupplier to read the digital value from
     */
    public DigitalBooleanSupplierIO(double debounceSeconds, BooleanSupplier supplier) {
        super(debounceSeconds);
        input = supplier;
    }

    /**
     * Gets the current value from the boolean supplier.
     * 
     * @return The boolean value from the supplier
     */
    @Override
    public boolean get() {
        return input.getAsBoolean();
    }
}
