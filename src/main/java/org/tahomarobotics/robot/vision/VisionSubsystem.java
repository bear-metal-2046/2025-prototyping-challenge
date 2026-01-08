package org.tahomarobotics.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class VisionSubsystem extends AbstractSubsystem {

    private final List<Limelight> limelights;

    VisionSubsystem() {
        this(new Limelight(VisionConstants.TEST_CAMERA));
    }

    private VisionSubsystem(Limelight... limelights){
        this.limelights = Arrays.asList(limelights);
    }

    @Override
    public void subsystemPeriodic() {
        for (Limelight limelight : limelights) {
            limelight.getEstimatedRobotPose().ifPresent(
                pose -> Logger.recordOutput("Vision/" + limelight.getName() + " Position", pose.poseEstimate().pose)
            );
        }
    }    
}