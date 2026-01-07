package org.tahomarobotics.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import org.tahomarobotics.robot.util.LimelightHelpers;
import org.tahomarobotics.robot.vision.VisionConstants.CameraConfiguration;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---


    public Limelight(CameraConfiguration config) {
        this.name = config.name();
        setCameraOffset(config);
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1 == null) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt1, this));
    }

    public void setCameraOffset(VisionConstants.CameraConfiguration cameraOffset){
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                cameraOffset.offset().getX(),
                cameraOffset.offset().getY(),
                cameraOffset.offset().getZ(),
                cameraOffset.offset().getRotation().getX(),
                cameraOffset.offset().getRotation().getY(),
                cameraOffset.offset().getRotation().getZ()
        );
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}

    public String getName() {
        return name;
    }
}
