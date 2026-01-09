package frc.robot.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class ControlBoard {
    public static void bindControls(Superstructure s) {
        // Shorter names to avoid long lines of accessing constants
        CommandXboxController driver = ControlBoardConstants.driver;
        CommandXboxController operator = ControlBoardConstants.operator;

        // Controls
        s.drive.setDefaultCommand(s.drive.teleopDrive(driver));
        
        driver.a().onTrue(s.exampleArm.prepScore());
        driver.x().onTrue(s.exampleIntake.unjam());
        driver.b().onTrue(s.stowAll());
        driver.y().onTrue(s.exampleShooter.prepShot());

        driver.leftTrigger(ControlBoardConstants.triggerThreshold).onTrue(s.intakeToShooter());

        driver.rightTrigger(ControlBoardConstants.triggerThreshold).onTrue(s.exampleShooter.fireWhenReady());
        driver.rightBumper().onTrue(s.exampleArm.score());

        driver.povUp().onTrue(s.prepClimb());
        driver.povDown().onTrue(s.exampleClimber.pull());

    }
}
