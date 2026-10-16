package frc.lib.util.visualization;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Represents the position of a mechanism relative to the robot
 */
public interface PositionNode {
    /**
     * @return The position to use as a base for all child nodes (end of arm, top of elevator, etc)
     */
    public Pose3d getAttachmentPose();
    /**
     * @return The position to use to visualize the state of the mechanism in AScope (base of arm or elevator, pointing in correct direction)
     */
    public Pose3d getVisualizationPose();

    /**
     * The parent node to use for mechanisms fixed to the robot
     */
    public static final PositionNode BASE = new PositionNode() {
        @Override
        public Pose3d getAttachmentPose() {return Pose3d.kZero;}

        @Override
        public Pose3d getVisualizationPose() {return Pose3d.kZero;}
    };
}
