package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.exampleIntake.ExampleIntake;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final ExampleIntake intake = new ExampleIntake();

    public Command intake() {
        return intake.intake().withName("Intake");
    }

    public Command spit() {
        return intake.spit().withName("Spit");
    }

    public Command stow() {
        return intake.stow().withName("Stow");
    }

    @Override
    public void log(String path) {
        Logger.log(path, "Intake", intake);
    }
}
