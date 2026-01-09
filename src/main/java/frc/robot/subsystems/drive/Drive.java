package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class Drive extends CommandSwerveDrivetrain implements Loggable {
    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.maxSpeed * 0.1).withRotationalDeadband(DriveConstants.maxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public Drive() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command teleopDrive(CommandXboxController controller) {
        return applyRequest(() ->
                teleopRequest.withVelocityX(-controller.getLeftY() * DriveConstants.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * DriveConstants.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * DriveConstants.maxAngularRate) // Drive counterclockwise with negative X (left)
            );
    }

    @Override
    public void log(String path) {
        logPose(path);
        logModules(path + "/Modules");
    }

    public void logPose(String path) {
        Logger.log(path, "Pose", getState().Pose);
    }

    public void logModules(String path) {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
        String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < modules.length; i++) {
            String name = moduleNames[i];
            SwerveModule<TalonFX, TalonFX, CANcoder> module = modules[i];
            Logger.log(path + "/" + name, "Drive/Stator Voltage", module.getDriveMotor().getMotorVoltage().getValue());
            Logger.log(path + "/" + name, "Drive/Supply Voltage", module.getDriveMotor().getSupplyVoltage().getValue());
            Logger.log(path + "/" + name, "Drive/Stator Current", module.getDriveMotor().getStatorCurrent().getValue());
            Logger.log(path + "/" + name, "Drive/Supply Current", module.getDriveMotor().getSupplyCurrent().getValue());
            Logger.log(path + "/" + name, "Drive/Velocity", module.getDriveMotor().getVelocity().getValue());
            Logger.log(path + "/" + name, "Drive/Acceleration", module.getDriveMotor().getAcceleration().getValue());
            Logger.log(path + "/" + name, "Drive/Temperature", module.getDriveMotor().getDeviceTemp().getValue());
            Logger.log(path + "/" + name, "Drive/Status", module.getDriveMotor().getMotorOutputStatus().getValue());

            Logger.log(path + "/" + name, "Steer/Stator Voltage", module.getSteerMotor().getMotorVoltage().getValue());
            Logger.log(path + "/" + name, "Steer/Supply Voltage", module.getSteerMotor().getSupplyVoltage().getValue());
            Logger.log(path + "/" + name, "Steer/Stator Current", module.getSteerMotor().getStatorCurrent().getValue());
            Logger.log(path + "/" + name, "Steer/Supply Current", module.getSteerMotor().getSupplyCurrent().getValue());
            Logger.log(path + "/" + name, "Steer/Velocity", module.getSteerMotor().getVelocity().getValue());
            Logger.log(path + "/" + name, "Steer/Acceleration", module.getSteerMotor().getAcceleration().getValue());
            Logger.log(path + "/" + name, "Steer/Temperature", module.getSteerMotor().getDeviceTemp().getValue());
            Logger.log(path + "/" + name, "Steer/Status", module.getSteerMotor().getMotorOutputStatus().getValue());
            Logger.log(path + "/" + name, "Steer/Position", module.getSteerMotor().getPosition().getValue());

            Logger.log(path + "/" + name, "Encoder/Absolute Position", module.getEncoder().getAbsolutePosition().getValue());
            Logger.log(path + "/" + name, "Encoder/Position", module.getEncoder().getPosition().getValue());
            Logger.log(path + "/" + name, "Encoder/Velocity", module.getEncoder().getVelocity().getValue());

            Logger.log(path + "/" + name, "Module/Speed", Units.MetersPerSecond.of(module.getCurrentState().speedMetersPerSecond));
            Logger.log(path + "/" + name, "Module/Angle", module.getCurrentState().angle);

        }
    }
}
