package org.tahomarobotics.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import org.tahomarobotics.robot.util.LimelightHelpers;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---


    public Limelight(String name) {
        this.name = name;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        LimelightHelpers.setCameraPose_RobotSpace();

        if (mt1 == null) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt1, this));
    }

    public void setCameraOffset(VisionConstants.CameraConfiguration cameraOffset){
        LimelightHelpers.setCameraPose_RobotSpace(
                getName(),
                cameraOffset.getX(),
                offset.getY(),
                offset.getZ(),
                offset.getRotation().getX(),
                offset.getRotation().getY(),
                offset.getRotation().getZ()
        );
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}

    public String getName() {
        return name;
    }
}
