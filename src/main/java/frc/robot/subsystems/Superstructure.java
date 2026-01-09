package frc.robot.subsystems;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.exampleArm.ExampleArm;
import frc.robot.subsystems.exampleClimber.ExampleClimber;
import frc.robot.subsystems.exampleIntake.ExampleIntake;
import frc.robot.subsystems.exampleShooter.ExampleShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Superstructure implements Loggable {
    public final Drive drive = new Drive();
    public final ExampleIntake exampleIntake = new ExampleIntake();
    public final ExampleShooter exampleShooter = new ExampleShooter();
    public final ExampleClimber exampleClimber = new ExampleClimber();
    public final ExampleArm exampleArm = new ExampleArm();

    @Override
    public void log(String path) {
        Logger.log(path, "Drive", drive);
        Logger.log(path, "Example Intake", exampleIntake);
        Logger.log(path, "Example Shooter", exampleShooter);
        Logger.log(path, "Example Climber", exampleClimber);
        Logger.log(path, "Example Arm", exampleArm);
    }

    public Command intakeToShooter() {
        return Commands.parallel(
            exampleIntake.intakeAndAutoStow(),
            exampleShooter.intakeToFeeder()  
        );
    }

    // Stow everything as we prepare to climb
    public Command prepClimb() {
        return Commands.parallel(
            exampleArm.stow(),
            exampleShooter.stow(),
            exampleIntake.stow(),
            exampleClimber.extend()
        );
    }

    public Command stowAll() {
        return Commands.parallel(
            exampleArm.stow(),
            exampleShooter.stow(),
            exampleIntake.stow(),
            exampleClimber.stow()
        );
    }

}
