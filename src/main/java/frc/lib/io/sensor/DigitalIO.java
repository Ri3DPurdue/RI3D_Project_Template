package frc.lib.io.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.lib.Util.logging.Loggable;

public abstract class DigitalIO implements Loggable {

    public static class DigitalIOOutputs {
        public boolean raw;
        public boolean debounced;
    }

    private final Debouncer debouncer;
    private final DigitalIOOutputs outputs;

    public DigitalIO(double debounceSeconds) {
		debouncer = new Debouncer(debounceSeconds, DebounceType.kBoth);
        outputs = new DigitalIOOutputs();
        update();
	}

	public abstract boolean get();

    public void update() {
        outputs.raw = get();
        outputs.debounced = debouncer.calculate(outputs.raw);
    }

	public boolean getDebounced() {
		return outputs.debounced;
	}

    @Override
    public void log(String path) {
        // TODO logging
        // Logger.processInputs(path, outputs);
    }
}
