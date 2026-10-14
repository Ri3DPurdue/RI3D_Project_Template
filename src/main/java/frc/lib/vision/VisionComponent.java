package frc.lib.vision;

import frc.lib.component.Component;
import frc.lib.util.logging.Logger;
import frc.lib.vision.VisionSubsystem.PoseEstimate;

public abstract class VisionComponent implements Component {
    private PoseEstimate latestEstimate;
    private final String name;

    public VisionComponent(String name) {
        this.name = name;
    }

    /**
     * Not to be called besides once per loop in this class' periodic method. Use {@link #getLatestEstimate()} to retrieve pose estimate instead.
     * 
     * @return The latest new pose estimate. Returns null if no new estimates since last call
     */
    protected abstract PoseEstimate updateLatestEstimate(); 

    public PoseEstimate getLatestEstimate() {
        return latestEstimate;
    }

    @Override
    public void log(String path) {
        Logger.log(path, name + "/Latest Estimate/Pose", latestEstimate.pose());
        Logger.log(path, name + "/Latest Estimate/Timestamp", latestEstimate.timestampSeconds());
    }

    @Override
    public void periodic() {
        latestEstimate = updateLatestEstimate();
    }
}
