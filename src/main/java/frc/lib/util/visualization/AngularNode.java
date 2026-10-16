package frc.lib.util.visualization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.component.ServoMotorComponent;

/**
 * Represents the position of a angular mechanism relative to the robot
 * <p>
 * NOTE: Angular mechanisms must rotate only in yaw. This means that when setting up the mechanism in AdvantageScope,
 * you must configure the mechanism such that it only rotates around the z-axis
 */
public class AngularNode implements PositionNode {
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
     * Creates an AngularNode to visualize the given servo readings
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param parentAttachmentToBaseOffset The transform from the parent's attachment point to this node's base
     * @param selfBaseToAttachmentOffset The transform from this node's base to this node's attachment point when the servo reads 0
     * @param servoReading The angle of the servo controlled mechanism
     */
    public AngularNode(
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        Supplier<Angle> servoReading
    ) {
        this.parent = parent;
        this.parentOffset = parentAttachmentToBaseOffset;
        this.servoOffset = () -> selfBaseToAttachmentOffset
            .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, servoReading.get().in(Units.Radians))));
    }

    /**
     * Creates an AngularNode to visualize the given servo
     * @param parent The parent node for this mechanism. If this mechanism is fixed relative to the robot, use {@link PositionNode.BASE}
     * @param parentAttachmentToBaseOffset The transform from the parent's attachment point to this node's base
     * @param selfBaseToAttachmentOffset The transform from this node's base to this node's attachment point when the servo reads 0
     * @param servo The servo controlling this mechanism
     */
    public AngularNode(
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        ServoMotorComponent<?> servo
    ) {
        this(
            parent, parentAttachmentToBaseOffset, selfBaseToAttachmentOffset, 
            () -> servo.getPosition()
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
