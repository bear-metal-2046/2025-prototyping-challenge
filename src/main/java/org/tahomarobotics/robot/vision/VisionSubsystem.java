package org.tahomarobotics.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class VisionSubsystem extends AbstractSubsystem {

    private final List<Limelight> limelights;
    private final Consumer<Limelight.EstimatedRobotPose> visionMeasurementConsumer;

    VisionSubsystem(Consumer<Limelight.EstimatedRobotPose> visionMeasurementConsumer){
        this(visionMeasurementConsumer, new Limelight(VisionConstants.TEST_CAMERA));
    }

    private VisionSubsystem(Consumer<Limelight.EstimatedRobotPose> visionMeasurementConsumer, Limelight... limelights){
        this.limelights = Arrays.asList(limelights);
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }

    @Override
    public void subsystemPeriodic() {
        for (Limelight limelight : limelights) {
            limelight.getEstimatedRobotPose().ifPresent(this::processVisionMeasurement);
        }
    }

    public void processVisionMeasurement(Limelight.EstimatedRobotPose pose) {
        Logger.recordOutput("Vision/" + pose.camera().getName() + " Position", pose.poseEstimate().pose);

        Logger.recordOutput("Vision/" + pose.camera().getName() + " Timestamp", pose.poseEstimate().timestampSeconds);
        visionMeasurementConsumer.accept(pose);
    }
}