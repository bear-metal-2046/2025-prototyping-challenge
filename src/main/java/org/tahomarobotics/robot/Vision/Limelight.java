package org.tahomarobotics.robot.Vision;

public class Limelight {

    // --- Fields and Constants ---
    private final String name;
    private Mode mode = Mode.MEGA_TAG_2;

    // --- Enum Definition ---

    public enum Mode {
        MEGA_TAG_2(1);

        final int pipelineIndex;

        Mode(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }
    }

    // --- Constructor ---

    public Limelight() {
        mode = Mode.MEGA_TAG_2;
        name = "Limelight Camera for testing";
    }

    public Limelight(Mode mode, String name) {
        this.mode = mode;
        this.name = name;
    }

}
