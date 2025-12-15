package frc.robot.subsystems.exampleClimber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.component.ServoMotorComponent;
import frc.lib.io.motor.rev.SparkBaseIO;
import frc.lib.io.motor.rev.SparkBaseSimIO;
import frc.lib.io.motor.setpoints.PositionSetpoint;
import frc.lib.mechanismSim.LinearSim;
import frc.lib.mechanismSim.SimObject;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;
import frc.robot.IDs;
import frc.robot.Robot;

public class ClimberConstants {
    // TODO MAKE DIFFERENT THAT COPY PASTE AND USE DISTANCES
    public static final Angle epsilonThreshold = Units.Degrees.of(10.0);
    public static final double gearing = 1.0;
    public static final DCMotor motor = DCMotor.getNeo550(2);

    // Create a converter to convert linear distances of the climber into rotations of the motor
    public static final DistanceAngleConverter converter = new DistanceAngleConverter(
            Units.Inches.of(2.0) // Diameter of pulley
            .plus(Units.Inches.of(0.25)) // Diameter of rope
            .div(2.0) // Divide by 2 to get radius
        );

    public static final Distance minDistance = Units.Inches.of(0.0);
    public static final Distance maxDistance = Units.Inches.of(12.0);

    public static final Distance extendDistance = Units.Inches.of(10.0);
    public static final Distance stowDistance = minDistance;
    public static final Distance pullDistance = Units.Inches.of(5.0);

    public static final PositionSetpoint extendSetpoint = new PositionSetpoint(converter.toAngle(extendDistance));
    public static final PositionSetpoint stowSetpoint = new PositionSetpoint(converter.toAngle(stowDistance));
    public static final PositionSetpoint pullSetpoint = new PositionSetpoint(converter.toAngle(pullDistance));

    public static final ServoMotorComponent<SparkBaseIO> getComponent() {
        SparkBaseIO io = getMotorIO();
        io.overrideLoggedUnits(converter.asAngleUnit(Units.Inches), converter.asAngularVelocityUnit(Units.InchesPerSecond), Units.Celsius);
        return new ServoMotorComponent<SparkBaseIO>(getMotorIO(), epsilonThreshold, converter.toAngle(stowDistance));
    }

    @SuppressWarnings("unchecked")
    public static final SparkBaseIO getMotorIO() {
        return Robot.isReal() 
            ? new SparkBaseIO(
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_PIVOT_MAIN.id, 
                Pair.of(IDs.INTAKE_PIVOT_FOLLOWER.id, false)
                )
            : new SparkBaseSimIO(
                getSimObject(),
                motor,
                MotorType.kBrushless, 
                getMainConfig(), 
                IDs.INTAKE_PIVOT_MAIN.id, 
                Pair.of(IDs.INTAKE_PIVOT_FOLLOWER.id, false)
            );
    }

    public static final SparkBaseConfig getMainConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(0.15)
            .d(0.15);
        return config;    
    }

    public static final SimObject getSimObject() {
        ElevatorSim system = 
            new ElevatorSim(
                motor, 
                gearing, 
                0.01, 
                0.2, 
                minDistance.in(Units.Meters), 
                minDistance.in(Units.Meters), 
                false,
                0.0, 
                0.0, 0.0);
        return new LinearSim(system, converter);
    }
}
