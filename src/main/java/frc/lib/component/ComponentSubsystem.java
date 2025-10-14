package frc.lib.component;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.logging.Loggable;

public class ComponentSubsystem extends SubsystemBase implements Loggable {
    private Map<String, Component> namedComponents = new HashMap<>();

    protected <C extends Component> C registerComponent(String name, C component) {
        namedComponents.put(name, component);
        return component;
    }

    @Override
    public void periodic() {
        for (Component componentWithName : namedComponents.values()) {
            componentWithName.periodic();
        }
    }

    @Override
    public void log(String subdirectory, String name) {
        for (String componentName : namedComponents.keySet()) {
            namedComponents.get(componentName).log(subdirectory + "/" + name, componentName);
        }
    }

    protected Command sequence(Command... commands) {
        Command cmd = Commands.sequence(commands);
        cmd.addRequirements(this); 
        return cmd;
    }

    protected Command parallel(Command... commands) {
        Command cmd = Commands.parallel(commands);
        cmd.addRequirements(this); 
        return cmd;
    }
}
