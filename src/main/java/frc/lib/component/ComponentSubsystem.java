package frc.lib.component;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ComponentSubsystem extends SubsystemBase {
    ArrayList<Component> components;

    protected <C extends Component> C registerComponent(C component) {
        components.add(component);
        return component;
    }

    @Override
    public void periodic() {
        for (Component component : components) {
            component.periodic();
        }
    }
}
