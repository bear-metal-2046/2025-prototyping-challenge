package org.tahomarobotics.robot.vision;

import edu.wpi.first.wpilibj.Notifier;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class VisionSubsystem extends AbstractSubsystem {

    private final Limelight limelight;
    private final Notifier offsetsWatcher;


    VisionSubsystem() {
        this(new Limelight(VisionConstants.TEST_CAMERA));
    }

    @Override
    public void subsystemPeriodic() {
        CameraMountEstimation.stream((pose) -> limelight.getEstimatedRobotPose().get(), this);
        limelight.getEstimatedRobotPose().ifPresent(
            pose -> Logger.recordOutput("Vision/" + limelight.getName() + " Position", pose.poseEstimate().pose)
        );
    }

    private VisionSubsystem(Limelight limelight){
        this.limelight = limelight;
        offsetsWatcher = new Notifier(CameraMountEstimation.stream(this.getLimelight()::getEstimatedRobotPose, this));
        offsetsWatcher.startPeriodic(Robot.defaultPeriodSecs);
    }

    public void applyEstimatedCameraOffsets() {
        org.tinylog.Logger.info("Applying estimated camera offsets.");
        limelight.applyEstimatedOffsets();
        org.tinylog.Logger.info("Finished applying estimated camera offsets.");
    }


    public Limelight getLimelight() {
        return limelight;
    }
}
