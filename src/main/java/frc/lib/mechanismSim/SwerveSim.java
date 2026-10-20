package frc.lib.mechanismSim;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class SwerveSim {
    private SimObject[] driveMotors;
    private SimObject[] angleMotors;
    private Supplier<Angle>[] encoders;
    private Supplier<Angle> gyro;
    private double mass;
    private double moi;
    private SwerveModuleSystem[] modules;
    private SwerveDriveKinematics kinematics;
    private double[][] states;
    private double lastUpdateTimeGlobal;
    @SuppressWarnings("unchecked")
    public SwerveSim(Mass mass, MomentOfInertia moi, SwerveModuleSystem... modules) {
        this.mass = mass.in(Kilograms);
        this.moi = moi.in(KilogramSquareMeters);
        this.modules = modules;
        states = new double[modules.length * 3 + 2][4];
        driveMotors = new SimObject[modules.length];
        angleMotors = new SimObject[modules.length];
        encoders = (Supplier<Angle>[]) new Object[modules.length];
        gyro = () -> Radians.of(states[modules.length * 3][0]);
        lastUpdateTimeGlobal = Timer.getFPGATimestamp();
        Translation2d[] translations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            driveMotors[i] = createDriveMotor(i);
            angleMotors[i] = createAngleMotor(i);
            encoders[i] = createEncoder(i);
            translations[i] = modules[i].offset();
        }
        this.kinematics = new SwerveDriveKinematics(translations);
    }

    private SimObject createDriveMotor(int i) {
        return new SimObject() {
            @Override
            public Angle getPosition() {
                return Radians.of(states[3 * i][0]);
            }
            
            @Override
            public AngularVelocity getVelocity() {
                return RadiansPerSecond.of(states[3 * i][1]);
            }

            @Override
            public Current getStatorCurrent() {
                return Amps.of(states[3 * i][2]);
            }

            @Override
            public void setVoltage(Voltage voltage) {
                states[3 * i][3] = voltage.in(Volts);
            }

            @Override
            protected void simulate(Time deltaTime) {}

            @Override
            public Time update() {
                globalSimulate();
                return super.update();
            }
        };
    }

    private SimObject createAngleMotor(int i) {
        return new SimObject() {
            @Override
            public Angle getPosition() {
                return Radians.of(states[3 * i + 1][0]);
            }

            @Override
            public AngularVelocity getVelocity() {
                return RadiansPerSecond.of(states[3 * i + 1][1]);
            }

            @Override
            public Current getStatorCurrent() {
                return Amps.of(states[3 * i + 1][2]);
            }

            @Override
            public void setVoltage(Voltage voltage) {
                states[3 * i + 1][3] = voltage.in(Volts);
            }

            @Override
            protected void simulate(Time deltaTime) {}
            
            @Override
            public Time update() {
                globalSimulate();
                return super.update();
            }
        };
    }

    private Supplier<Angle> createEncoder(int i) {
        return () -> Radians.of(states[3 * i + 2][0]);
    }
   
    public SimObject[] getDriveMotors() {
        return driveMotors;
    }

    public SimObject[] getAngleMotors() {
        return angleMotors;
    }

    public Supplier<Angle>[] getEncoders() {
        return encoders;
    }

    public Supplier<Angle> getGyro() {
        return gyro;
    }

    private void globalSimulate() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime  - lastUpdateTimeGlobal;
        lastUpdateTimeGlobal = currentTime; 
        double chassisForceX = 0;
        double chassisForceY = 0;
        double chassisTorque = 0;
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            var wheelRadius = module.wheelRadius().in(Meters);
            var driveCurrent = module.driveMotor().getCurrent(states[3 * i][1] * module.driveGearReduction(), states[3 * i][3]);
            var driveCurrentLimit = module.driveStatorLimit().in(Amps);
            driveCurrent = MathUtil.clamp(driveCurrent, -driveCurrentLimit, driveCurrentLimit);
            states[3 * i][2] = driveCurrent;
            var driveTorque = module.driveMotor().getTorque(driveCurrent);
            var force = driveTorque * module.driveGearReduction() / wheelRadius;

            var angleCurrent = module.angleMotor().getCurrent(states[3 * i + 1][1] * module.angleGearReduction(), states[3 * i + 1][3]);
            var angleCurrentLimit = module.angleStatorLimit().in(Amps);
            angleCurrent = MathUtil.clamp(angleCurrent, -angleCurrentLimit, angleCurrentLimit);
            states[3 * i + 1][2] = angleCurrent;
            var angleTorque = module.angleMotor().getTorque(angleCurrent) * module.angleGearReduction();
            var angleAccel = angleTorque / module.wheelMOI().in(KilogramSquareMeters);
            states[3 * i + 1][1] += angleAccel * dt;
            states[3 * i + 1][0] += states[3 * i + 1][1] * dt;
            states[3 * i + 2][0] = states[3 * i + 1][0];

            var forceX = force * Math.cos(states[3 * i + 1][0]);
            var forceY = force * Math.sin(states[3 * i + 1][0]);
            var torque = module.offset().getX() * forceY - module.offset.getY() * forceX;
            chassisForceX += forceX;
            chassisForceY += forceY;
            chassisTorque += torque;
        }
        var accelerationX = chassisForceX / mass;
        var accelerationY = chassisForceY / mass;
        var accelerationW = chassisTorque / moi;
        states[3 * modules.length + 1][0] += accelerationX * dt;
        states[3 * modules.length + 1][1] += accelerationY * dt;
        states[3 * modules.length + 1][2] += accelerationW * dt;
        states[3 * modules.length][0] += states[3 * modules.length + 1][2] * dt;
        var moduleStates = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                states[3 * modules.length + 1][0], 
                states[3 * modules.length + 1][1], 
                states[3 * modules.length + 1][2])
        );
        for (int i = 0; i < modules.length; i++) {
            states[3 * i][1] = moduleStates[i].speedMetersPerSecond / modules[i].wheelRadius().in(Meters);
            states[3 * i][0] += states[3 * i][1] * dt;
        }
    }

    public static record SwerveModuleSystem(
        DCMotor driveMotor,
        Current driveStatorLimit,
        double driveGearReduction,
        Distance wheelRadius,
        DCMotor angleMotor,
        Current angleStatorLimit,
        MomentOfInertia wheelMOI,
        double angleGearReduction,
        Translation2d offset
    ) {}

}
