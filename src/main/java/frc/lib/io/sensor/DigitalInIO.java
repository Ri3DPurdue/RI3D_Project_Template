package frc.lib.io.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInIO extends DigitalIO {
    private final DigitalInput input;

    public DigitalInIO(double debounceSeconds, int port) {
        super(debounceSeconds);
        input = new DigitalInput(port);
    }

    @Override
    public boolean get() {
        return input.get();
    }
    
}
