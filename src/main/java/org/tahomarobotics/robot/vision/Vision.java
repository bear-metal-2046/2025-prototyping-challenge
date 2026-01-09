package org.tahomarobotics.robot.vision;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class Vision {
    final VisionSubsystem vision;

    public Vision(BiConsumer<Limelight.EstimatedRobotPose, Boolean> visionMeasurementConsumer) {
        this(new VisionSubsystem(visionMeasurementConsumer));
    }

    private Vision(VisionSubsystem vision) {
        this.vision = vision;
    }
}
