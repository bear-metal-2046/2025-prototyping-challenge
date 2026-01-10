package org.tahomarobotics.robot.vision;

import java.util.function.Consumer;

public class Vision {
    final VisionSubsystem vision;

    public Vision(Consumer<Limelight.EstimatedRobotPose> visionMeasurementConsumer) {
        this(new VisionSubsystem(visionMeasurementConsumer));
    }

    private Vision(VisionSubsystem vision) {
        this.vision = vision;
    }
}
