package frc.lib.component;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.sensor.DigitalIO;

public class DigitalIOComponent implements Component {
    private final DigitalIO io;

    public DigitalIOComponent(DigitalIO digitalIO) {
        io = digitalIO;
    }

    @Override
    public void periodic() {
        io.update();
    }

    @Override
    public void log(String subdirectory, String name) {
        io.log(subdirectory, name);
    }

    public boolean get() {
        return io.get();
    }

    public boolean getDebounced() {
        return io.getDebounced();
    }
    
    public Command stateWait(boolean state) {
        return Commands.waitUntil(() -> state == get());
    }
    
    public Command stateWaitDebounced(boolean state) {
        return Commands.waitUntil(() -> state == getDebounced());
    }
}
