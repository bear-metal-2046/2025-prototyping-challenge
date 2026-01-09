package org.tahomarobotics.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Radians;

import org.dyn4j.geometry.Transform;
import org.tahomarobotics.robot.util.LimelightHelpers;
import org.tahomarobotics.robot.util.LimelightHelpers.LimelightResults;
import org.tahomarobotics.robot.vision.VisionConstants.CameraConfiguration;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    // --- Constructor ---


    public Limelight(CameraConfiguration config) {
        this.name = config.name();
        setCameraOffset(config.offset());
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPoseMegaTag1() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (mt1 == null || mt1.tagCount < 1) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt1, this));
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPoseMegaTag2() {
        Optional<EstimatedRobotPose> mt1 = getEstimatedRobotPoseMegaTag1();
        if (!mt1.isPresent()) {
            return Optional.empty();
        }

        Angle yaw = mt1.get().poseEstimate().pose.getRotation().getMeasure();
        LimelightHelpers.SetRobotOrientation(name, yaw.in(Radians), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (mt2 == null || mt2.tagCount < 1) {
            return Optional.empty();
        }
        return Optional.of(new EstimatedRobotPose(mt2, this));
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
