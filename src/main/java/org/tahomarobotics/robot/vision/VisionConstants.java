package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

public class VisionConstants {

    public static CameraConfiguration Test_Camera;

    static {
        Test_Camera = new CameraConfiguration(
                "Limelight",
                new Transform3d(
                        new Translation3d(inchesToMeters(4.5), inchesToMeters(0), inchesToMeters(8.66142)),
                        new Rotation3d(degreesToRadians(0), degreesToRadians(0), degreesToRadians(0))
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
