package frc.robot.controlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;

public class ControlBoard {
    public static void bindControls(Superstructure s) {
        // Shorter names to avoid long lines of accessing constants
        CommandXboxController driver = ControlBoardConstants.driver;
        CommandXboxController operator = ControlBoardConstants.operator;

        // Controls
        s.drive.setDefaultCommand(s.drive.teleopDrive(driver));
        driver.start().onTrue(Commands.runOnce(() -> s.drive.resetPose(new Pose2d())));

        driver.a().onTrue(s.exampleArm.prepScore());
        driver.x().onTrue(s.exampleIntake.unjam());
        driver.b().onTrue(s.stowAll());
        driver.y().onTrue(s.exampleShooter.prepShot());

        driver.leftTrigger(ControlBoardConstants.triggerThreshold).onTrue(s.intakeToShooter());

        driver.rightTrigger(ControlBoardConstants.triggerThreshold).onTrue(s.exampleShooter.fireWhenReady());
        driver.rightBumper().onTrue(s.exampleArm.score());

        driver.povUp().onTrue(s.prepClimb());
        driver.povDown().onTrue(s.exampleClimber.pull());

        SmartDashboard.putData("Reset Pose", Commands.runOnce(() -> s.drive.resetPose(Pose2d.kZero)));
        SmartDashboard.putData("Prep Score", s.exampleArm.prepScore());
        SmartDashboard.putData("Unjam Intake", s.exampleIntake.unjam());
        SmartDashboard.putData("Stow All", s.stowAll());
        SmartDashboard.putData("Prep Shot", s.exampleShooter.prepShot());
        SmartDashboard.putData("Intake and Feed", s.intakeToShooter());
        SmartDashboard.putData("Fire When Ready", s.exampleShooter.fireWhenReady());
        SmartDashboard.putData("Score", s.exampleArm.score());
        SmartDashboard.putData("Prep Climb", s.prepClimb());
        SmartDashboard.putData("Climb", s.exampleClimber.pull());
    }
}
