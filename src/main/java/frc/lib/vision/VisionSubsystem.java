package frc.lib.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.logging.Loggable;
import frc.lib.util.logging.Logger;

public class VisionSubsystem<V extends VisionComponent> extends SubsystemBase implements Loggable{
    private final V camera;
    private final BiConsumer<Pose2d, Double> visionUpdater;

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionEstimate, V cam) {
        camera = cam;
        visionUpdater = addVisionEstimate;
    }

    // Returns null if no new estimate since last call of this method
    public PoseEstimate getLatestEstimate() {
        return camera.updateLatestEstimate();
    }

    @Override
    public void periodic() {
        addLatestEstimate();
    }

    public boolean shouldAcceptEstimate(PoseEstimate estimate) {
        return estimate.exists();
    }

    public void addLatestEstimate() {
        PoseEstimate latestEstimate = getLatestEstimate();
        if (shouldAcceptEstimate(latestEstimate)) {
            visionUpdater.accept(latestEstimate.pose(), latestEstimate.timestampSeconds());
        }
    }


    @Override
    public void log(String path) {
        Logger.log(path, camera);
    }

      /** Holds an estimated position from a vison system. */
    public static record PoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double averageDistance,
      double averageArea,
      boolean exists) {}
}
