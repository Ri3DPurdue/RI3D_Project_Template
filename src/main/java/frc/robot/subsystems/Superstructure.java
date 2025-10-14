package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.logging.Loggable;
import frc.robot.subsystems.ExampleIntake.ExampleIntake;

public class Superstructure implements Loggable {
    public final ExampleIntake intake = new ExampleIntake();

    public Command intake() {
        return intake.intake();
    }

    @Override
    public void log(String subdirectory, String name) {
        intake.log(subdirectory + "/" + name, "Intake");
    }
}
