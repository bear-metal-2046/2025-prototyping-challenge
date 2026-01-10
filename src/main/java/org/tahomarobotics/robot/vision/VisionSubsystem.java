package org.tahomarobotics.robot.vision;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class VisionSubsystem extends AbstractSubsystem {

    private final Limelight limelight;

    VisionSubsystem() {
        this(new Limelight(VisionConstants.TEST_CAMERA));
    }

    @Override
    public void subsystemPeriodic() {
        limelight.getEstimatedRobotPose().ifPresent(
            pose -> Logger.recordOutput("Vision/" + limelight.getName() + " Position", pose.poseEstimate().pose)
        );
    }

    private VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
    }

    public Limelight getLimelight() {
        return limelight;
    }
}
