package org.tahomarobotics.robot.arm;

import org.tinylog.Logger;

public class Arm implements AutoCloseable {
    private final ArmSubsystem arm;

    public Arm() {
        this(new ArmSubsystem());
        Logger.info("Creating an instance of Arm...");
    }

    Arm(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void close() throws Exception {
    }
}
