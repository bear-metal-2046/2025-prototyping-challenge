package org.tahomarobotics.robot.vision;

import edu.wpi.first.wpilibj.Notifier;

public class Vision {
    final VisionSubsystem vision;

    public Vision() {
        this(new VisionSubsystem());
    }

    private Vision(VisionSubsystem vision) {
        this.vision = vision;
    }
}
