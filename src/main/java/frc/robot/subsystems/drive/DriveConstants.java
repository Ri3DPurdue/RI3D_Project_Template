package frc.robot.subsystems.drive;

import java.util.HashMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.component.SwerveModuleComponent;
import frc.lib.io.motor.ctre.TalonFXIO;
import frc.lib.io.motor.ctre.TalonFXIOSim;
import frc.lib.io.sensor.encoder.CANCoderIO;
import frc.lib.io.sensor.encoder.CANCoderIOSim;
import frc.lib.io.sensor.gyro.Pigeon2IO;
import frc.lib.io.sensor.gyro.Pigeon2IOSim;
import frc.lib.mechanismSim.SwerveSim;
import frc.lib.mechanismSim.SwerveSim.SwerveModuleSystem;
import frc.lib.util.ConfigUtil;
import frc.robot.IDs;
import frc.robot.Robot;

public class DriveConstants {
    public static final LinearVelocity maxSpeed = Units.MetersPerSecond.of(4);
    public static final AngularVelocity maxAngularRate = Units.RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity

    
    public static final double driveGearRatio = 1.0;
    public static final double steerGearRatio = 1.0;
    public static final Distance wheelRadius = Units.Inches.of(2);

    public static final Angle steerEpsilon = Units.Degrees.of(5);
    public static final LinearVelocity driveEpsilon = Units.FeetPerSecond.of(0.1);

    public static final DCMotor driveMotor = DCMotor.getKrakenX60(1);
    public static final DCMotor steerMotor = DCMotor.getKrakenX60(1);

    public static final Distance drivebaseLength = Units.Inches.of(28);
    public static final Distance drivebaseWidth = Units.Inches.of(28);

    public static final Mass robotMass = Units.Pound.of(100);
    public static final MomentOfInertia robotMOI = Units.KilogramSquareMeters.of(20);
    public static final MomentOfInertia wheelMOI = Units.KilogramSquareMeters.of(0.1);

    public static final Translation2d frontLeftOffset = new Translation2d(drivebaseLength.div( 2), drivebaseWidth.div( 2));
    public static final Translation2d frontRightOffset = new Translation2d(drivebaseLength.div( 2), drivebaseWidth.div(-2));
    public static final Translation2d backLeftOffset = new Translation2d(drivebaseLength.div(-2), drivebaseWidth.div( 2));
    public static final Translation2d backRightOffset = new Translation2d(drivebaseLength.div(-2), drivebaseWidth.div(-2));

    private static final SwerveSim sim = new SwerveSim(
        robotMass,
        robotMOI,
        newModule(frontLeftOffset),
        newModule(frontRightOffset),
        newModule(backLeftOffset),
        newModule(backRightOffset)
    );


    private static final SwerveModuleSystem newModule(Translation2d offset) {
        return new SwerveModuleSystem(
            driveMotor,
            Units.Amps.of(ConfigUtil.safeStatorCurrentLimitAmps),
            driveGearRatio,
            wheelRadius,
            steerMotor,
            Units.Amps.of(ConfigUtil.safeCurrentLimitAmps),
            wheelMOI,
            steerGearRatio,
            offset
        );
    }

    public static final HashMap<
        String,
        Pair<
            SwerveModuleComponent<
                TalonFXIO,
                TalonFXIO, 
                CANCoderIO
            >,
            Translation2d
        >
    > getModules() {
        HashMap<
            String,
            Pair<
                SwerveModuleComponent<
                    TalonFXIO,
                    TalonFXIO,
                    CANCoderIO
                >,
                Translation2d
            >
        > modules = new HashMap<>();

        modules.put("Front Left", new Pair<>(
            getModuleComponent(IDs.FRONT_LEFT_DRIVE, IDs.FRONT_LEFT_STEER, IDs.FRONT_LEFT_ENCODER, 0),
            frontLeftOffset
        ));

        modules.put("Front Right", new Pair<>(
            getModuleComponent(IDs.FRONT_RIGHT_DRIVE, IDs.FRONT_RIGHT_STEER, IDs.FRONT_RIGHT_ENCODER, 0),
            frontRightOffset
        ));

        modules.put("Back Left", new Pair<>(
            getModuleComponent(IDs.BACK_LEFT_DRIVE, IDs.BACK_LEFT_STEER, IDs.BACK_LEFT_ENCODER, 0),
            backLeftOffset
        ));

        modules.put("Back Right", new Pair<>(
            getModuleComponent(IDs.BACK_RIGHT_DRIVE, IDs.BACK_RIGHT_STEER, IDs.BACK_RIGHT_ENCODER, 0),
            backRightOffset
        ));

        return modules;
    }

    @SuppressWarnings({ "unchecked" })
    private static final SwerveModuleComponent<TalonFXIO, TalonFXIO, CANCoderIO> getModuleComponent(IDs driveID, IDs steerID, IDs encoderID, int index) {
            TalonFXIO drive = Robot.isReal() ?
            new TalonFXIO(
                driveID.id,
                driveID.bus,
                getDriveConfiguration()
            ) :
            new TalonFXIOSim(
                driveID.id, driveID.bus, getDriveConfiguration(),
                sim.getDriveMotors()[index],
                driveGearRatio
            );

            TalonFXIO steer = Robot.isReal() ?
            new TalonFXIO(
                steerID.id,
                steerID.bus,
                getSteerConfiguration()
            ) :
            new TalonFXIOSim(
                steerID.id,
                steerID.bus,
                getSteerConfiguration(),
                sim.getAngleMotors()[index],
                steerGearRatio
            );

            CANCoderIO encoder = Robot.isReal() ?
            new CANCoderIO(encoderID.id, encoderID.bus) :
            new CANCoderIOSim(encoderID.id, encoderID.bus, sim.getEncoders()[index]);

            return new SwerveModuleComponent<>(drive, driveEpsilon, steer, steerEpsilon, encoder, wheelRadius);
    }

    public static final Pigeon2IO getGyroIO(IDs gyroID) {
        return Robot.isReal() ?
            new Pigeon2IO(gyroID.id, gyroID.bus) :
            new Pigeon2IOSim(gyroID.id, gyroID.bus, sim.getGyro());
    }

    public static final TalonFXConfiguration getDriveConfiguration() {
        TalonFXConfiguration conf = ConfigUtil.getSafeFXConfig(driveGearRatio, 50, ConfigUtil.safeStatorCurrentLimitAmps);

        conf.Slot0.kV = 1.0;
        conf.Slot0.kP = 1.0;
        conf.Slot0.kI = 0.001;

        return conf;
    }

    public static final TalonFXConfiguration getSteerConfiguration() {
        TalonFXConfiguration conf = ConfigUtil.getSafeFXConfig(steerGearRatio);

        conf.Slot0.kP = 2.0;
        conf.Slot0.kI = 0.001;

        return conf;
    }
}
