package frc.lib.drive.swerve;

import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.component.ComponentSubsystem;
import frc.lib.component.GyroIOComponent;
import frc.lib.component.SwerveModuleComponent;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.sensor.encoder.EncoderIO;
import frc.lib.io.sensor.gyro.GyroIO;

public class SwerveDrive<D extends MotorIO, S extends MotorIO, E extends EncoderIO> extends ComponentSubsystem {
    private final SwerveModuleComponent<D, S, E>[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;
    private final GyroIOComponent gyroscope;
    private final LinearVelocity maxModuleSpeed;

    @SuppressWarnings("unchecked")
    public SwerveDrive(
        Map<String, Pair<SwerveModuleComponent<D, S, E>, Translation2d>> modules,
        String gyroName,
        GyroIO gyro,
        LinearVelocity maxModuleSpeed,
        Pose2d initialPose
    ) {
        Translation2d[] moduleTranslations = new Translation2d[modules.size()];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.size()];

        this.modules = new SwerveModuleComponent[modules.size()];
        int idx = 0;

        for (var e : modules.entrySet()) {
            this.modules[idx] = e.getValue().getFirst();
            moduleTranslations[idx] = e.getValue().getSecond();

            positions[idx] = new SwerveModulePosition(
                Units.Meter.of(0),
                Rotation2d.fromRadians(e.getValue().getFirst().getSteerComponent().getPosition().in(Units.Radians))
            );

            this.registerComponent(e.getKey(), e.getValue().getFirst());

            idx++;
        }

        this.gyroscope = registerComponent(gyroName, new GyroIOComponent(gyro)); 
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
        this.odometry = new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromRadians(this.gyroscope.getYaw().in(Units.Radians)),
            positions,
            initialPose
        );
        this.maxModuleSpeed = maxModuleSpeed;
    }

    public SwerveDrive(
        Map<String, Pair<SwerveModuleComponent<D, S, E>, Translation2d>> modules,
        String gyroName,
        GyroIO gyro,
        LinearVelocity maxModuleSpeed
    ) {
        this(modules, gyroName, gyro, maxModuleSpeed, new Pose2d());
    }

    public SwerveDrive(
        SwerveModuleComponent<D, S, E> frontLeftModule,
        SwerveModuleComponent<D, S, E> frontRightModule,
        SwerveModuleComponent<D, S, E> backLeftModule,
        SwerveModuleComponent<D, S, E> backRightModule,
        Distance drivebaseWidth,
        Distance drivebaseLength,
        String gyroName,
        GyroIO gyro,
        LinearVelocity maxModuleSpeed,
        Pose2d initialPose
    ) {
    this(
        Map.of(
            "Front Left",  new Pair<>(frontLeftModule,  new Translation2d(drivebaseLength.div( 2), drivebaseWidth.div( 2))),
            "Front Right", new Pair<>(frontRightModule, new Translation2d(drivebaseLength.div( 2), drivebaseWidth.div(-2))),
            "Back Left",   new Pair<>(backLeftModule,   new Translation2d(drivebaseLength.div(-2), drivebaseWidth.div( 2))),
            "Back Right",  new Pair<>(backRightModule,  new Translation2d(drivebaseLength.div(-2), drivebaseWidth.div(-2)))
        ),
        gyroName,
        gyro,
        maxModuleSpeed,
        initialPose
    );
    }

    public SwerveDrive(
        SwerveModuleComponent<D, S, E> frontLeftModule,
        SwerveModuleComponent<D, S, E> frontRightModule,
        SwerveModuleComponent<D, S, E> backLeftModule,
        SwerveModuleComponent<D, S, E> backRightModule,
        Distance drivebaseWidth,
        Distance drivebaseLength,
        String gyroName,
        GyroIO gyro,
        LinearVelocity maxModuleSpeed
    ) {
    this(
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule,
        drivebaseWidth,
        drivebaseLength,
        gyroName,
        gyro,
        maxModuleSpeed,
        new Pose2d()
    );
    }

    @Override
    public void periodic() {
        super.periodic();

        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            positions[i] =  new SwerveModulePosition(
                module.getDriveDistance(),
                Rotation2d.fromRadians(module.getSteerAngle().in(Units.Radians))
            );
        }

        odometry.update(Rotation2d.fromRadians(gyroscope.getYaw().in(Units.Radians)), positions);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxModuleSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].applySetpoint(states[i].angle, Units.MetersPerSecond.of(states[i].speedMetersPerSecond));
        }
    }

    /*
     * Because chassis speeds are meters per second, this method is private
     * because it treats the speeds as a duty cycle
     */
    private void driveVoltage(ChassisSpeeds speeds) {
        // Units don't matter...
        // https://i.pinimg.com/originals/26/b2/50/26b250a738ea4abc7a5af4d42ad93af0.jpg

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);

        for (int i = 0; i < modules.length; i++) {
            modules[i].applySetpoint(states[i].angle, Units.Volts.of(states[i].speedMetersPerSecond * 12));
        }
    }

    public void driveRobotRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        drive(new ChassisSpeeds(x, y, omega));
    }

    public void driveRobotRelative(double xDutyCycle, double yDutyCycle, double omegaDutyCycle) {
        driveVoltage(new ChassisSpeeds(xDutyCycle, yDutyCycle, omegaDutyCycle));
    }

    public void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, omega,
            Rotation2d.fromRadians(getRobotPose().getRotation().getRadians())
        ));
    }

    public void driveFieldRelative(double xDutyCycle, double yDutyCycle, double omegaDutyCycle) {
        driveVoltage(ChassisSpeeds.fromFieldRelativeSpeeds(
            xDutyCycle, yDutyCycle, omegaDutyCycle,
            Rotation2d.fromRadians(getRobotPose().getRotation().getRadians())
        ));
    }

    public Pose2d getRobotPose() {
        return odometry.getEstimatedPosition();
    }

    public void setRobotPose(Pose2d pose) {
        odometry.resetPose(pose);
    }

    public void addVisionMeasurement(Pose2d estimate, Time timestamp) {
        odometry.addVisionMeasurement(estimate, timestamp.in(Units.Seconds));
    }

    public void addVisionMeasurement(Pose2d estimate, double timestampSeconds) {
        odometry.addVisionMeasurement(estimate, timestampSeconds);
    }
}
