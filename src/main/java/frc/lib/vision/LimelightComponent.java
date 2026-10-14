package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.vision.VisionSubsystem.PoseEstimate;

public class LimelightComponent extends VisionComponent {
    private final NetworkTable table;

    public LimelightComponent(String name, int pipeline) {
        super(name);
        table = NetworkTableInstance.getDefault().getTable(name);
        table.getEntry("pipline").setInteger(pipeline);
    }

    @Override
    protected PoseEstimate updateLatestEstimate() {
        double[] raw = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[11]);
        return new PoseEstimate(
            new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5])),
            raw[6] / 1000,
            (int) raw[7],
            Timer.getFPGATimestamp() - raw[9], 
            raw[10],
            hasTargets());
    }

    /**
     * @return whether the limelight sees a target
     */
    public boolean hasTargets() {
        return table.getEntry("tv").getInteger(0) == 1;
    }
    
}
