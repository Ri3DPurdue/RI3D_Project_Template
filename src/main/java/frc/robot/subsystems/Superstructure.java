package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleIntake.ExampleIntake;

public class Superstructure {
    public final ExampleIntake intake = new ExampleIntake();

    public Command intake() {
        return intake.intake();
    }
}
