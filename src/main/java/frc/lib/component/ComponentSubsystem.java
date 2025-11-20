package frc.lib.component;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.exampleIntake1.util.logging.Loggable;
import frc.robot.subsystems.exampleIntake1.util.logging.Logger;

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
    public void log(String name) {
        for (Map.Entry<String, Component> namedComponent : namedComponents.entrySet()) {
            Logger.log(name, namedComponent.getKey(), namedComponent.getValue());
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

    protected Command command(Command command) {
        command.addRequirements(this); 
        return command;
    }
}
