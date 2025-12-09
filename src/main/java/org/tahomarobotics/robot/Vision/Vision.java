package org.tahomarobotics.robot.Vision;

public class Vision {
    private final VisionSubsystem visionSubsystem;

    public Vision() {
        this(new VisionSubsystem());
    }

    private Vision(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }
}
