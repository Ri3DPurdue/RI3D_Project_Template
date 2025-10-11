package frc.lib.io.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public abstract class DigitalIO {
    private final Debouncer debouncer;
    private boolean lastRawValue;
    private boolean lastDebouncedValue;

    public DigitalIO(double debounceSeconds) {
		debouncer = new Debouncer(debounceSeconds, DebounceType.kBoth);
        update();
	}

	public abstract boolean get();

    public void update() {
        lastRawValue = get();
        lastDebouncedValue = debouncer.calculate(lastRawValue);
    }

	public boolean getDebounced() {
		return lastDebouncedValue;
	}
}
