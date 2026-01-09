package org.tahomarobotics.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;

import org.dyn4j.geometry.Transform;
import org.tahomarobotics.robot.util.LimelightHelpers;
import org.tahomarobotics.robot.vision.VisionConstants.CameraConfiguration;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---


    public Limelight(CameraConfiguration config) {
        this.name = config.name();
        setCameraOffset(config.offset());
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1 == null || mt1.tagCount < 1) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt1, this));
    }

    public void setCameraOffset(Transform3d cameraOffset){
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                cameraOffset.getX(),
                cameraOffset.getY(),
                cameraOffset.getZ(),
                cameraOffset.getRotation().getX(),
                cameraOffset.getRotation().getY(),
                cameraOffset.getRotation().getZ()
        );
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}

    public String getName() {
        return name;
    }
}
