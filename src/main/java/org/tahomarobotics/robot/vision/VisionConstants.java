package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;

public class VisionConstants {

    public static CameraConfiguration TEST_CAMERA;
    public static final int CAMERA_ERROR_AVERAGE_WINDOW = 10;

    static {
        TEST_CAMERA = new CameraConfiguration(
                "limelight",
                new Transform3d(
                        new Translation3d(Inches.of(4.5), Inches.of(0), Inches.of(8.66142)),
                        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
                        )
                );
    }


    public record CameraConfiguration(String name, Transform3d offset){
        public double[] getTransformArray() {
            return new double[] {
                    offset.getTranslation().getMeasureX().in(Units.Meters),
                    offset.getTranslation().getMeasureY().in(Units.Meters),
                    offset.getTranslation().getMeasureZ().in(Units.Meters),
                    offset.getRotation().getMeasureX().in(Units.Degree),
                    offset.getRotation().getMeasureY().in(Units.Degree),
                    offset.getRotation().getMeasureZ().in(Units.Degree)
            };
        }

    }




}
