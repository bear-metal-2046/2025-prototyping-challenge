package org.tahomarobotics.robot.vision;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class VisionSubsystem extends AbstractSubsystem {

    private final Limelight limelight;

    VisionSubsystem() {
        this(new Limelight("Limelight"));
    }

    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput("Vision/Position", limelight.getEstimatedRobotPose());
    }

    private VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
    }
}
