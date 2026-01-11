/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.util.LimelightHelpers;
import org.tinylog.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class CameraMountEstimation {
    private static final HashMap<String, double[]> mountPositions = new HashMap<>();

    public static Consumer<Limelight.EstimatedRobotPose> stream(Consumer<Limelight.EstimatedRobotPose> original, VisionSubsystem vision) {
        // Whether to estimate the next AprilTag update from each camera.
        Map<String, Boolean> estimate =
                Stream.of(VisionConstants.TEST_CAMERA.name())
                        .collect(Collectors.toMap(s -> s, $ -> false));

        // Initialize all of our inputs on SmartDashboard
        SmartDashboard.putData(
                "Estimate Camera Positions", Commands.runOnce(() -> estimate.replaceAll((n, v) -> true)).ignoringDisable(true)
        );

        SmartDashboard.putData(
                "Update Camera Positions", Commands.runOnce(() -> vision.applyEstimatedCameraOffsets()).ignoringDisable(true)
        );


        SmartDashboard.putNumber("Actual Chassis Pose X (Meters)", 0);
        SmartDashboard.putNumber("Actual Chassis Pose Y (Meters)", 0);
        SmartDashboard.putNumber("Actual Chassis Pose Heading (Degrees)", 0);

        return estimatedRobotPose -> {
            if (estimate.getOrDefault(estimatedRobotPose.camera().getName(), false)) {
                double x = SmartDashboard.getNumber("Actual Chassis Pose X (Meters)", 0);
                double y = SmartDashboard.getNumber("Actual Chassis Pose Y (Meters)", 0);
                double h = Units.degreesToRadians(SmartDashboard.getNumber("Actual Chassis Pose Heading (Degrees)", 0));

                Pose2d actualChassisPose = new Pose2d(x, y, new Rotation2d(h));
                LimelightHelpers.RawFiducial[] tags = LimelightHelpers.getRawFiducials(vision.getLimelight().getName());
                for (LimelightHelpers.RawFiducial tag : tags) {
                    // Get the camera-to-target transform
                    Transform3d cameraToTargetTranspose = new Transform3d(new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0)), LimelightHelpers.getCameraPose3d_TargetSpace(vision.getLimelight().getName()));

                    // Get the expected field-pose of the corresponding apriltag on the field
                    Pose3d targetPose = VisionConstants.FIELD_LAYOUT.getTagPose(tag.id).orElseThrow();


                    // Subtract the camera-to-target transform from the target field-pose to get the expected field-to-camera position
                    Pose3d cameraPose = targetPose.plus(cameraToTargetTranspose.inverse());

                    // Subtract the field-to-camera pose from the actual field-to-chassis pose to get the expected camera-to-chassis transform
                    Transform3d cameraToRobotTransform = cameraPose.minus(new Pose3d(actualChassisPose));

                    // Decompose the transformation into its components
                    Translation3d t = cameraToRobotTransform.getTranslation();
                    Rotation3d r = cameraToRobotTransform.getRotation();

                    // Publish the results to SmartDashboard
                    mountPositions.put(
                            estimatedRobotPose.camera().getName(), new double[]{
                                    t.getX(),
                                    t.getY(),
                                    t.getZ(),
                                    r.getX(),
                                    r.getY(),
                                    r.getZ()}
                    );
                    Logger.info(mountPositions);
                    estimate.put(estimatedRobotPose.camera().getName(), false);
                }
            }
            original.accept(estimatedRobotPose);
        };
    }

    public static double[] getEstimatedMountPose(String cameraName) {
        return mountPositions.get(cameraName);
    }
}