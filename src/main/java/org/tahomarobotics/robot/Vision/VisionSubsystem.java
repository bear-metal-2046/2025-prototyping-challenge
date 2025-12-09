package org.tahomarobotics.robot.Vision;

public class VisionSubsystem {

    private final Limelight limelight;

    public VisionSubsystem() {
        this(new Limelight());
    }

    public VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
    }
}
