package frc.lib.util.visualization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.component.ServoMotorComponent;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;
import frc.lib.util.logging.Logger;

/**
 * Represents the position of a linear mechanism relative to the robot
 * <p>
 * NOTE: Linear mechanisms must only in extend in x. This means that when setting up the mechanism in AdvantageScope,
 * you must configure the mechanism such that it only extends along the x-axis
 */
public class LinearNode implements PositionNode {
    /**
     * The parent node for this mechanism
     */
    private final PositionNode parent;

    /**
     * Subscriber to read the overriden servo value from NT
     */
    private final DoubleSubscriber servoReadingOverride;

    /**
     * Subscriber to read whether we are overriding the servo reading from NT
     */
    private final BooleanSubscriber overrideServo;
     
    /**
     * The offset from the parent nodeto this node when the servo reads 0
     */
    private Supplier<Transform3d> parentOffset;
    
    /**
     * The offset caused by the servo reading
     */
    private Supplier<Transform3d> servoOffset;

    /**
     * Creates a LinearNode to visualize the given servo readings
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param offsetFromParent The transform from the parent to this node when the servo reads 0
     * @param servoReading The position of the servo controlled mechanism
     */
    public LinearNode(
        String nodeName,
        PositionNode parent,
        Transform3d offsetFromParent,
        Supplier<Distance> servoReading
    ) {
        this.parent = parent;
        DoubleSubscriber[] parentOffsets = new DoubleSubscriber[6];
        parentOffsets[0] = Logger.tunable(nodeName + "/Parent Offset/X (m)", offsetFromParent.getX());
        parentOffsets[1] = Logger.tunable(nodeName + "/Parent Offset/Y (m)", offsetFromParent.getY());
        parentOffsets[2] = Logger.tunable(nodeName + "/Parent Offset/Z (m)", offsetFromParent.getZ());
        parentOffsets[3] = Logger.tunable(nodeName + "/Parent Offset/Roll (deg)", Math.toDegrees(offsetFromParent.getRotation().getX()));
        parentOffsets[4] = Logger.tunable(nodeName + "/Parent Offset/Pitch (deg)", Math.toDegrees(offsetFromParent.getRotation().getY()));
        parentOffsets[5] = Logger.tunable(nodeName + "/Parent Offset/Yaw (deg)", Math.toDegrees(offsetFromParent.getRotation().getZ()));
        servoReadingOverride = Logger.tunable(nodeName + "/Servo Reading Override (m)", 0.0);
        overrideServo = Logger.tunable(nodeName + "/Override Servo Reading?", false);
        this.parentOffset = () -> new Transform3d(
            parentOffsets[0].get(),
            parentOffsets[1].get(),
            parentOffsets[2].get(),
            new Rotation3d(
                Math.toRadians(parentOffsets[3].get()),
                Math.toRadians(parentOffsets[4].get()),
                Math.toRadians(parentOffsets[5].get())
            )
        );
        this.servoOffset = () -> {
            if (overrideServo.get()) {
                return new Transform3d(servoReadingOverride.get(), 0, 0, Rotation3d.kZero);
            }
            return new Transform3d(servoReading.get().in(Units.Meters), 0, 0, Rotation3d.kZero);
        };
    }

    /**
     * Creates a LinearNode to visualize the given servo
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param offsetFromParent The transform from the parent to this node when the servo reads 0
     * @param servo The servo controlling this mechanism
     * @param converter The converter between the servo's angle and the mechanism's linear position
     */
    public LinearNode(
        String nodeName,
        PositionNode parent,
        Transform3d offsetFromParent,
        ServoMotorComponent<?> servo,
        DistanceAngleConverter converter
    ) {
        this(
            nodeName, parent, offsetFromParent, 
            () -> converter.toDistance(servo.getPosition())
        );
    }

    @Override
    public Pose3d getPose() {
        return parent.getPose().plus(parentOffset.get()).plus(servoOffset.get());
    }
}