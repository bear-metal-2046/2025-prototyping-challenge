package org.tahomarobotics.robot.Vision;

public class VisionSubsystem {

    private final Limelight limelight;

    VisionSubsystem() {
        this(new Limelight());
    }

    private VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
    }
}
