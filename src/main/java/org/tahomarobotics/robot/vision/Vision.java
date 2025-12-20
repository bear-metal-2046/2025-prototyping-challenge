package org.tahomarobotics.robot.vision;

public class Vision {
    private final VisionSubsystem visionSubsystem;

    public Vision() {
        this(new VisionSubsystem());
    }

    private Vision(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }
}
