package frc.lib.io.sensor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.lib.Util.logging.Loggable;
import frc.lib.Util.logging.Logger;

public abstract class DigitalIO implements Loggable {

    public static class DigitalIOOutputs implements Loggable {
        public boolean raw;
        public boolean debounced;

        @Override
        public void log(String path) {
            Logger.log(path, "Raw Ouput", raw);
            Logger.log(path, "Debounced Output", debounced);
        }
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
        Logger.log(path, outputs);
    }
}
