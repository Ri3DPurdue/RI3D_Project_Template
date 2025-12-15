package frc.robot.subsystems;

import frc.robot.subsystems.exampleArm.ExampleArm;
import frc.robot.subsystems.exampleClimber.ExampleClimber;
import frc.robot.subsystems.exampleIntake.ExampleIntake;
import frc.robot.subsystems.exampleShooter.ExampleShooter;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final ExampleIntake exampleIntake = new ExampleIntake();
    public final ExampleShooter exampleShooter = new ExampleShooter();
    public final ExampleClimber exampleClimber = new ExampleClimber();
    public final ExampleArm exampleArm = new ExampleArm();

    @Override
    public void log(String path) {
        Logger.log(path, "Example Intake", exampleIntake);
        Logger.log(path, "Example Shooter", exampleShooter);
        Logger.log(path, "Example Climber", exampleClimber);
        Logger.log(path, "Example Arm", exampleArm);
    }
}
