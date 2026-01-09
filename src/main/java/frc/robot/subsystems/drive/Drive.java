package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controlBoard.ControlBoardConstants;

public class Drive extends CommandSwerveDrivetrain{
    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.maxSpeed * 0.1).withRotationalDeadband(DriveConstants.maxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public Drive() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    public Command teleopDrive() {
        return applyRequest(() ->
                teleopRequest.withVelocityX(-ControlBoardConstants.driver.getLeftY() * DriveConstants.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-ControlBoardConstants.driver.getLeftX() * DriveConstants.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-ControlBoardConstants.driver.getRightX() * DriveConstants.maxAngularRate) // Drive counterclockwise with negative X (left)
            );
    }
}
