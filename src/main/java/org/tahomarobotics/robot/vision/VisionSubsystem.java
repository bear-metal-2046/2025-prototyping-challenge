package org.tahomarobotics.robot.vision;

public class VisionSubsystem {

    private final Limelight limelight;

    VisionSubsystem() {
        this(new Limelight("Limelight Camera"));
    }

    private VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
    }
}
