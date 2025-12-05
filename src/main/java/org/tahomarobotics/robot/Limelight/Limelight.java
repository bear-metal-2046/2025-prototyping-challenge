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

//camera class
package org.tahomarobotics.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.json.JSONArray;
import org.json.JSONObject;
import org.photonvision.PhotonUtils;
import org.tahomarobotics.robot.util.persistent.CalibrationData;

import java.util.Optional;

import static org.tahomarobotics.robot.Vision.VisionConstants.Coral;
import static org.tahomarobotics.robot.Vision.VisionConstants.JSONArraykey;

public class Limelight {

    // --- Fields and Constants ---

    private final VisionConstants.CameraConfiguration configuration;
    private final CalibrationData<double[]> mountPositionCalibration;

    private final String name;
    private Mode mode = Mode.MEGA_TAG_2;

    private int prioritizedTag;

    // --- Enum Definition ---

    public enum Mode {
        CORAL_DETECTION(0),
        // ISOLATED_APRIL_TAG(1),
        MEGA_TAG_2(1);

        final int pipelineIndex;

        Mode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }
    }

    // --- Constructor ---

    /**
     * @param configuration The configuration position of the cameras
     * @param mode          What the cameras are looking for, such as april tags and coral.
     * @param name          The name of the camera
     */
    public Limelight(VisionConstants.CameraConfiguration configuration, Mode mode, String name) {
        this.configuration = configuration;
        this.mode = mode;
        this.name = name;
        mountPositionCalibration = new CalibrationData<>(configuration.name() + "Calibration", configuration.getTransformArray());
    }

    // --- Getters ---

    public String getName() {
        return name;
    }

    private Transform3d getOffset() {
        return configuration.transform();
    }
// ---something---

    // --- Vision Processing Methods ---

    public Optional<EstimatedRobotPose> getEstimatedPose () {
        if (mode != Mode.MEGA_TAG_2) {
            return Optional.empty();
        }
        LLH.PoseEstimate poseEstimate = LLH.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
        JSONArray standardDeviationsJSON = new JSONObject(LLH.getJSONDump(getName())).getJSONArray(JSONArraykey);

        // Convert JSONArray to double[]
        double[] standardDeviations = standardDeviationsJSON.toList().stream().mapToDouble(d -> (double) d).toArray();

        Vector<N3> standardDeviationsVector = VecBuilder.fill(
                standardDeviations[0],
                standardDeviations[1],
                Units.degreesToRadians(standardDeviations[4])
        );

        return Optional.of(new EstimatedRobotPose(poseEstimate, standardDeviationsVector, this));
    }


    public Optional<Translation2d> getRobotRelativeCoralLocation (Mode coralDetection){
        if (mode != Mode.CORAL_DETECTION) {
            return Optional.empty();
        }
        if (!LLH.getTV(getName()) || !LLH.getDetectorClass(getName()).equalsIgnoreCase(Coral)) {
            return Optional.empty();
        }
        return getRobotRelativeTargetTranslation();
    }
    public Optional<Translation2d> getRobotRelativeTargetTranslation () {

        double targetX = LLH.getTX(getName());
        double targetY = LLH.getTY(getName());
        double targetHeight;
        switch (mode) {
            case CORAL_DETECTION -> targetHeight = Units.inchesToMeters(2.25);
            case MEGA_TAG_2 -> {
                Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose((int) LLH.getFiducialID(getName()));
                if (tagPose.isEmpty()) {
                    return Optional.empty();
                }
                targetHeight = tagPose.get().getZ();
            }
            default -> {
                return Optional.empty();
            }
        }

        // --- Calculate distance from camera to target --- //
        double distanceFromCamera = PhotonUtils.calculateDistanceToTargetMeters(
                getOffset().getZ(),                          // Camera height
                targetHeight,                               // Target height
                getOffset().getRotation().getY(),           // Camera pitch (radians)
                Units.degreesToRadians(targetY)             // Vertical angle to target (radians)
        );

        // --- Convert polar to cartesian (camera-relative) --- //
        double angleToTarget = Units.degreesToRadians(targetX); // Horizontal angle
        double x = distanceFromCamera * Math.cos(angleToTarget);
        double y = distanceFromCamera * Math.sin(angleToTarget);

        Translation2d cameraToTarget = new Translation2d(x, y);
        Translation2d robotToCamera = getOffset().getTranslation().toTranslation2d();

        // --- Apply camera offset to get robot-relative position --- //
        return Optional.of(robotToCamera.plus(cameraToTarget));
    }
    public record EstimatedRobotPose(LLH.PoseEstimate poseEstimate, Vector<N3> stdDevs, Limelight camera) {}
}