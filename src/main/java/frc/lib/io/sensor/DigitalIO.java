package frc.lib.io.sensor;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.lib.io.logging.Loggable;

public abstract class DigitalIO implements Loggable {
    @AutoLog
    public static class DigitalIOOutputs {
        public boolean raw;
        public boolean debounced;
    }

    private final Debouncer debouncer;
    private final DigitalIOOutputsAutoLogged outputs;

    public DigitalIO(double debounceSeconds) {
		debouncer = new Debouncer(debounceSeconds, DebounceType.kBoth);
        outputs = new DigitalIOOutputsAutoLogged();
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
    public void log(String subdirectory, String name) {
        String path = subdirectory + "/" + name;

        Logger.processInputs(path, outputs);
    }
}
