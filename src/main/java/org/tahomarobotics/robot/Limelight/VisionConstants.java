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

package org.tahomarobotics.robot.Limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

/** Constants for the {@link "Vision"} subsystem. */
public class VisionConstants {

    public static final String Coral = "coral";
    public static final String JSONArraykey = "stddevs_mt2";
    public record CameraConfiguration(String name, Transform3d transform) {
        public double[] getTransformArray() {
            return new double[]{
                    transform.getTranslation().getX(),
                    transform.getTranslation().getY(),
                    transform.getTranslation().getZ(),
                    transform.getRotation().getX(),
                    transform.getRotation().getY(),
                    transform.getRotation().getZ()
            };
        }
    }
    // AprilTag Field Layout
    // TODO: Convert this to be a mapping from field name (or some other identifier) to a layout so we can support
    //  multiple fields.
    /**
     * The AprilTag field layout for whatever field we are on. See {@link "AprilTagCamera"}'s JavaDoc for more information
     * on how to calculate this properly.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Cameras


    public static CameraConfiguration LIMELIGHT = new CameraConfiguration("Limelight", new Transform3d(
            new Translation3d(Meters.of(0.150248), Meters.of(0.228600), Meters.of(0.821795)),
            new Rotation3d(Degrees.zero(), Degrees.of(32.7), Degrees.zero())));
}