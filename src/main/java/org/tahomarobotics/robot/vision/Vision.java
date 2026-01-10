package org.tahomarobotics.robot.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class Vision {
    final VisionSubsystem vision;

    public Vision(BiConsumer<Limelight.EstimatedRobotPose, Vector<N3>> visionMeasurementConsumer) {
        this(new VisionSubsystem(visionMeasurementConsumer));
    }

    private Vision(VisionSubsystem vision) {
        this.vision = vision;
    }
}
