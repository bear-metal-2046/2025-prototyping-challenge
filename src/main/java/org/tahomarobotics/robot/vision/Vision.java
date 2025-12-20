package org.tahomarobotics.robot.vision;

public class Vision {
    final VisionSubsystem vision;

    public Vision() {
        this(new VisionSubsystem());
    }

    private Vision(VisionSubsystem vision) {
        this.vision = vision;
    }
}
