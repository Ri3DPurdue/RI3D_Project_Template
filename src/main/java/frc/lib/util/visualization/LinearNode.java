package frc.lib.util.visualization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.component.ServoMotorComponent;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;

/**
 * Represents the position of a linear mechanism relative to the robot
 * <p>
 * NOTE: Linear mechanisms must rotate only in extend in x. This means that when setting up the mechanism in AdvantageScope,
 * you must configure the mechanism such that it only extends towards the x-axis
 */
public class LinearNode implements PositionNode {
    /**
     * The parent node for this mechanism
     */
    private final PositionNode parent;

    /**
     * The offset from the parent node's attachment point to this node's base
     */
    private Transform3d parentOffset;
    
    /**
     * The offset from this node's base to this node's attachment point
     */
    private Supplier<Transform3d> servoOffset;

    /**
     * Creates a LinearNode to visualize the given servo readings
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param parentAttachmentToBaseOffset The transform from the parent's attachment point to this node's base
     * @param selfBaseToAttachmentOffset The transform from this node's base to this node's attachment point when the servo reads 0
     * @param servoReading The position of the servo controlled mechanism
     */
    public LinearNode(
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        Supplier<Distance> servoReading
    ) {
        this.parent = parent;
        this.parentOffset = parentAttachmentToBaseOffset;
        this.servoOffset = () -> selfBaseToAttachmentOffset
            .plus(new Transform3d(servoReading.get().in(Units.Meters), 0, 0, Rotation3d.kZero));
    }

    /**
     * Creates a LinearNode to visualize the given servo
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param parentAttachmentToBaseOffset The transform from the parent's attachment point to this node's base
     * @param selfBaseToAttachmentOffset The transform from this node's base to this node's attachment point when the servo reads 0
     * @param servo The servo controlling this mechanism
     * @param converter The converter between the servo's angle and the mechanism's linear position
     */
    public LinearNode(
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        ServoMotorComponent<?> servo,
        DistanceAngleConverter converter
    ) {
        this(
            parent, parentAttachmentToBaseOffset, selfBaseToAttachmentOffset, 
            () -> converter.toDistance(servo.getPosition())
        );
    }

    @Override
    public Pose3d getAttachmentPose() {
        return parent.getAttachmentPose().plus(parentOffset).plus(servoOffset.get());
    }

    @Override
    public Pose3d getVisualizationPose() {
        return parent.getAttachmentPose().plus(parentOffset).plus(
            new Transform3d(Translation3d.kZero, servoOffset.get().getRotation()));
    }
}