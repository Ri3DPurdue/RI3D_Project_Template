package frc.lib.component;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.logging.Loggable;

public class ComponentSubsystem extends SubsystemBase implements Loggable {
    private ArrayList<Pair<Component, String>> componentsWithName = new ArrayList<>();

    protected <C extends Component> C registerComponent(C component, String name) {
        componentsWithName.add(Pair.of(component, name));
        return component;
    }

    @Override
    public void periodic() {
        for (Pair<Component, String> componentWithName : componentsWithName) {
            componentWithName.getFirst().periodic();
        }
    }

    @Override
    public void log(String subdirectory, String name) {
        for (Pair<Component, String> componentWithName : componentsWithName) {
            componentWithName.getFirst().log(subdirectory + "/" + name, componentWithName.getSecond());;
        }
    }
}
