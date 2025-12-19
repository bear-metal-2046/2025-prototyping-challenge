package org.tahomarobotics.robot.vision;

import java.util.Optional;

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

        if (mt1 == null) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt1, this));
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}

    public String getName() {
        return name;
    }
}
