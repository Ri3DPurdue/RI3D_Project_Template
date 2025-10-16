package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Util.logging.Loggable;
import frc.robot.subsystems.ExampleIntake.ExampleIntake;

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
        intake.log(path, "Intake");
    }
}
