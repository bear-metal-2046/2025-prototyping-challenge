package org.tahomarobotics.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Notifier;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.util.LimelightHelpers;
import org.tahomarobotics.robot.vision.VisionConstants.CameraConfiguration;
import org.tinylog.Logger;

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

    public void applyEstimatedOffsets() {
        double[] mountPosition = CameraMountEstimation.getEstimatedMountPose(this.getName());
        if (mountPosition == null) {
            Logger.error("Could not retrieve camera mount position for " + this.getName());
            return;
        }
        //xyz rotation
        CameraConfiguration newConfig = new CameraConfiguration(this.getName(), new Transform3d(
                new Translation3d(
                        mountPosition[0],
                        mountPosition[1],
                        mountPosition[2]
                ),
                new Rotation3d(
                        mountPosition[3],
                        mountPosition[4],
                        mountPosition[5]
                )
        ));
        setCameraOffset(newConfig);
    }

    public record EstimatedRobotPose(LimelightHelpers.PoseEstimate poseEstimate, Limelight camera) {}

    public String getName() {
        return name;
    }
}
