package org.tahomarobotics.robot.vision;

import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.TriConsumer;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends AbstractSubsystem {

    private final List<Limelight> limelights;
    private final TriConsumer<Pose2d, Double, Vector<N3>> visionMeasurementConsumer;

    VisionSubsystem(TriConsumer<Pose2d, Double, Vector<N3>> visionMeasurementConsumer){
        this(visionMeasurementConsumer, new Limelight(VisionConstants.TEST_CAMERA));
    }

    private VisionSubsystem(TriConsumer<Pose2d, Double, Vector<N3>> visionMeasurementConsumer, Limelight... limelights){
        this.limelights = Arrays.asList(limelights);
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }

    @Override
    public void subsystemPeriodic() {
        for (Limelight limelight : limelights) {
            limelight.getEstimatedRobotPoseMegaTag2()
                .ifPresent(positionMeasurement -> processVisionMeasurement(positionMeasurement)
            );
        }
    }

    public void processVisionMeasurement(Limelight.EstimatedRobotPose pose) {
        Logger.recordOutput("Vision/" + pose.camera().getName() + " Position", pose.poseEstimate().pose);

        // Get all tag IDs used in the estimation and log
        int[] tagIDs = Arrays.asList(pose.poseEstimate().rawFiducials).stream().mapToInt(fiducial -> fiducial.id).toArray();
        Logger.recordOutput("Vision/ " + pose.camera().getName() + " IDs Seen", tagIDs);

        Logger.recordOutput("Vision/" + pose.camera().getName() + " Timestamp", pose.poseEstimate().timestampSeconds);
        visionMeasurementConsumer.accept(pose.poseEstimate().pose, pose.poseEstimate().timestampSeconds, VisionConstants.MEGATAG2_STANDARD_DEVIATIONS);
    }
}