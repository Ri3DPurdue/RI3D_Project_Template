package frc.lib.mechanismSim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
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
    private double[][] states;
    private double lastUpdateTimeGlobal;
    public SwerveSim(Mass mass, MomentOfInertia moi, SwerveModuleSystem... modules) {
        states = new double[modules.length * 3 + 1][4];
        driveMotors = new SimObject[modules.length];
        angleMotors = new SimObject[modules.length];
        lastUpdateTimeGlobal = Timer.getFPGATimestamp();
        for (int i = 0; i < modules.length; i++) {
            driveMotors[i] = createDriveMotor(i, modules[i].wheelRadius().in(Meters));
            angleMotors[i] = createAngleMotor(i);
        }
    }

    private SimObject createDriveMotor(int i, double wheelRadius) {
        return new SimObject() {
            @Override
            public Angle getPosition() {
                return Radians.of(states[3 * i][0] / wheelRadius);
            }
            
            @Override
            public AngularVelocity getVelocity() {
                return RadiansPerSecond.of(states[3 * i][1] / wheelRadius);
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

    private void globalSimulate() {
        double currentTime = Timer.getFPGATimestamp();
        double delta = currentTime  - lastUpdateTimeGlobal;
        lastUpdateTimeGlobal = currentTime; 
        // TODO: Actual sim math
    }

    public static record SwerveModuleSystem(
        DCMotor driveMotor,
        double driveGearReduction,
        Distance wheelRadius,
        DCMotor angleMotor,
        double angleGearReduction,
        Translation2d offset
    ) {}

}
