package frc.lib.util.visualization;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.lib.component.ServoMotorComponent;
import frc.lib.util.UnitsUtil.DistanceAngleConverter;

/**
 * Represents the position of a {@link ServoMotorComponent} relative to the robot
 */
public class PositionNode {
    private final Optional<PositionNode> parent;
    private final Transform3d offset;
    private final ServoMotorComponent<?> servo;
    private final Transform3d attachmentOffset;
    private Transform3d state;
    private final Optional<DistanceAngleConverter> converter;
    private final int servoAxis;

    /**
     * Create a position node for a given servo which controls an angle
     * @param parent The parent node for this node. If the servo doesn't have a parent node (it is fixed relative to the robot),
     * then pass in an empty optional
     * @param offsetFromParent The offset from the parent's attachment location to this node's starting location
     * @param servo The servo component that updates this node's state
     * @param defaultAttachmentOffset The offset between this node's start and end when the servo reads zero
     * @param servoAxis Which axis the servo is in {x, y, z, roll, pitch, yaw} -> {0, 1, 2, 3, 4, 5}.
     * @throws UnsupportedOperationException if the servoAxis is X, Y, or Z, use the other constructor if
     * the servo controls a distance
     */
    public PositionNode(
        Optional<PositionNode> parent,
        Transform3d offsetFromParent,
        ServoMotorComponent<?> servo,
        Transform3d defaultAttachmentOffset,
        ServoAxis servoAxis
    ) {
        this.parent = parent;
        this.offset = offsetFromParent;
        this.servo = servo;
        this.attachmentOffset = defaultAttachmentOffset;
        this.servoAxis = servoAxis.ordinal();
        this.converter = Optional.empty();
        if (servoAxis == ServoAxis.X || servoAxis == ServoAxis.Y || servoAxis == ServoAxis.Z) {
            throw new UnsupportedOperationException(
                "If the servo controls a distance, you must use the other constructor to pass in the distance converted");
        }
    }

    /**
     * Create a position node for a given servo which controls a distance
     * @param parent The parent node for this node. If the servo doesn't have a parent node (it is fixed relative to the robot),
     * then pass in an empty optional
     * @param offsetFromParent The offset from the parent's attachment location to this node's starting location
     * @param servo The servo component that updates this node's state
     * @param defaultAttachmentOffset The offset between this node's start and end when the servo reads zero
     * @param servoAxis Which axis the servo is in {x, y, z, roll, pitch, yaw} -> {0, 1, 2, 3, 4, 5}.
     * @param converter Converts the servo's angle reading into a distance traveled measurement
     * @throws UnsupportedOperationException if the servoAxis is Roll, Pitch, or Yaw, use the other constructor if
     * the servo controls a rotation
     */
    public PositionNode(
        Optional<PositionNode> parent,
        Transform3d offsetFromParent,
        ServoMotorComponent<?> servo,
        Transform3d defaultAttachmentOffset,
        ServoAxis servoAxis,
        DistanceAngleConverter converter
    ) {
        this.parent = parent;
        this.offset = offsetFromParent;
        this.servo = servo;
        this.attachmentOffset = defaultAttachmentOffset;
        this.servoAxis = servoAxis.ordinal();
        this.converter = Optional.of(converter);
        if (servoAxis == ServoAxis.ROLL || servoAxis == ServoAxis.PITCH || servoAxis == ServoAxis.YAW) {
            throw new UnsupportedOperationException(
                "If the servo controls an angle, you must use the other constructor and not pass in a converter");
        }
    }

    /**
     * @return The position to use as an origin for a child node
     */
    private Pose3d getAttachmentPose() {
        return getVisualizationPose().plus(new Transform3d(state.getTranslation(), Rotation3d.kZero));
    }
    
    /**
     * @return The position to use as a visualization for displaying the state of the servo
     */
    public Pose3d getVisualizationPose() {
        updateState();
        return parent
            .map(PositionNode::getAttachmentPose)
            .orElse(Pose3d.kZero)
            .plus(offset)
            .plus(new Transform3d(Translation3d.kZero, state.getRotation()));
    }
    
    private void updateState() {
        double[] reading = new double[]{0, 0, 0, 0, 0, 0};
        if (converter.isEmpty()) {
            reading[servoAxis] = servo.getPosition().in(Units.Radians);
        } else {
            reading[servoAxis] = converter.get().toDistance(servo.getPosition()).in(Units.Meters);
        }
        state = attachmentOffset.plus(new Transform3d(
            reading[0],
            reading[1],
            reading[2],
            new Rotation3d(
                reading[4],
                reading[5],
                reading[6]
            )
        ));
    }

    public enum ServoAxis {
        X,
        Y,
        Z,
        ROLL,
        PITCH,
        YAW
    }
}
