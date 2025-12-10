package org.tahomarobotics.robot.Vision;

public class vision {
    private final VisionSubsystem visionSubsystem;

    public vision() {
        this(new VisionSubsystem());
    }

    private vision(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }
}
