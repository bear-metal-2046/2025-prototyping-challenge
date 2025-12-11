package org.tahomarobotics.robot.vision;

import org.tahomarobotics.robot.util.LimelightHelpers;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---


    public Limelight(String name) {
        this.name = name;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return new EstimatedRobotPose(mt1, this);
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}


}
