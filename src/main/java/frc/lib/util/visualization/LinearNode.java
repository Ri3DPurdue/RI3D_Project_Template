package frc.lib.util.visualization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
     * Subscribers to read the parent offsets from NT (x, y, z, roll, pitch, yaw)
     */
    private final DoubleSubscriber[] parentOffsets;

    /**
     * Subscribers to read the attachment offsets from NT (x, y, z, roll, pitch, yaw)
     */
    private final DoubleSubscriber[] attachmentOffsets;

    /**
     * Subscriber to read the overriden servo value from NT
     */
    private final DoubleSubscriber servoReadingOverride;

    /**
     * Subscriber to read whether we are overriding the servo reading from NT
     */
    private final BooleanSubscriber overrideServo;
    
    /**
     * The offset from the parent node's attachment point to this node's base
     */
    private Supplier<Transform3d> parentOffset;
    
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
        String nodeName,
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        Supplier<Distance> servoReading
    ) {
        this.parent = parent;
        parentOffsets = new DoubleSubscriber[6];
        parentOffsets[0] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/X (m)", parentAttachmentToBaseOffset.getX());
        parentOffsets[1] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/Y (m)", parentAttachmentToBaseOffset.getY());
        parentOffsets[2] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/Z (m)", parentAttachmentToBaseOffset.getZ());
        parentOffsets[4] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/Roll (rad)", parentAttachmentToBaseOffset.getRotation().getX());
        parentOffsets[3] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/Pitch (rad)", parentAttachmentToBaseOffset.getRotation().getY());
        parentOffsets[5] = Logger.tunable("TuneableNumbers/" + nodeName + "ParentOffset/Yaw (rad)", parentAttachmentToBaseOffset.getRotation().getZ());
        attachmentOffsets = new DoubleSubscriber[6];
        attachmentOffsets[0] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/X (m)", selfBaseToAttachmentOffset.getX());
        attachmentOffsets[1] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/Y (m)", selfBaseToAttachmentOffset.getY());
        attachmentOffsets[2] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/Z (m)", selfBaseToAttachmentOffset.getZ());
        attachmentOffsets[4] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/Roll (rad)", selfBaseToAttachmentOffset.getRotation().getX());
        attachmentOffsets[3] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/Pitch (rad)", selfBaseToAttachmentOffset.getRotation().getY());
        attachmentOffsets[5] = Logger.tunable("TuneableNumbers/" + nodeName + "Attachment Offset/Yaw (rad)", selfBaseToAttachmentOffset.getRotation().getZ());
        servoReadingOverride = Logger.tunable("TuneableNumbers/" + nodeName + "/Servo Reading Override (m)", 0.0);
        overrideServo = Logger.tunable("TuneableNumbers/" + nodeName + "/Override Servo Reading?", false);
        this.parentOffset = () -> new Transform3d(
            parentOffsets[0].get(),
            parentOffsets[1].get(),
            parentOffsets[2].get(),
            new Rotation3d(
                parentOffsets[3].get(),
                parentOffsets[4].get(),
                parentOffsets[5].get()
            )
        );
        this.servoOffset = () -> {
            Transform3d attachmentOffset = new Transform3d(
                attachmentOffsets[0].get(),
                attachmentOffsets[1].get(),
                attachmentOffsets[2].get(),
                new Rotation3d(
                    attachmentOffsets[3].get(),
                    attachmentOffsets[4].get(),
                    attachmentOffsets[5].get()
                )
            );
            if (overrideServo.get()) {
                return attachmentOffset.plus(
                    new Transform3d(servoReadingOverride.get(), 0, 0, Rotation3d.kZero)
                );
            }
            return attachmentOffset.plus(
                new Transform3d(servoReading.get().in(Units.Meters), 0, 0, Rotation3d.kZero)
            );
        };
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
        String nodeName,
        PositionNode parent,
        Transform3d parentAttachmentToBaseOffset,
        Transform3d selfBaseToAttachmentOffset,
        ServoMotorComponent<?> servo,
        DistanceAngleConverter converter
    ) {
        this(
            nodeName, parent, parentAttachmentToBaseOffset, selfBaseToAttachmentOffset, 
            () -> converter.toDistance(servo.getPosition())
        );
    }

    @Override
    public Pose3d getAttachmentPose() {
        return parent.getAttachmentPose().plus(parentOffset.get()).plus(servoOffset.get());
    }

    @Override
    public Pose3d getVisualizationPose() {
        return parent.getAttachmentPose().plus(parentOffset.get()).plus(
            new Transform3d(Translation3d.kZero, servoOffset.get().getRotation()));
    }
}