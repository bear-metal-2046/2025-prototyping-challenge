package org.tahomarobotics.robot.vision;


import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import org.tahomarobotics.robot.util.LimelightHelpers;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---

    public Limelight() {
        name = "Limelight Camera for testing";
    }

    public Limelight(String name) {
        this.name = name;
    }

    public EstimatedRobotPose getEstimatedRobotPose() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return new EstimatedRobotPose(mt1, this);
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}


}
