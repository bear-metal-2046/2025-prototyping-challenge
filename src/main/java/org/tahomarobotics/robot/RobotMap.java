/*
 * MIT License
 *
 * Copyright (c) 2025 Bear Metal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.tahomarobotics.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {

    // arm
    public static final int ARM_TOP_MOTOR = 0;
    public static final int ARM_BOTTOM_MOTOR = 1;
    public static final int ELBOW_ENCODER = 2;
    public static final int WRIST_ENCODER = 3;
    //Hardware IDs (find proper assigned numbers from Design Leads)
    //Source: Lines 287-289 NEW_SUBSYSTEM_GUIDE.md
    public static final int ENDEFFECTOR_MOTOR = 4;
    public static final int ARM_TOP_ENCODER = 20;
    public static final int ARM_BOTTOM_ENCODER = 21;
    public static final int ARM_WRIST_ENCODER = 22;

    // elevator
    public static final int ELEVATOR_MOTOR_LEFT = 10;
    public static final int ELEVATOR_MOTOR_RIGHT = 11;
    public static final int ELEVATOR_ENCODER = 12;

    public static final String CANBUS_NAME = "Canivore";

    public static final int PIGEON = 0;

    public final static moduleId FRONT_LEFT_MODULE = new moduleId(
            "Front Left Module", 1, 11, 21);
    public final static moduleId FRONT_RIGHT_MODULE = new moduleId(
            "Front Right Module", 2, 12, 22);
    public final static moduleId BACK_LEFT_MODULE = new moduleId(
            "Front Left Module", 3, 13, 23);
    public final static moduleId BACK_RIGHT_MODULE = new moduleId(
            "Front Right Module", 4, 14, 24);



    public record moduleId(String moduleName, int driveId, int steerId, int cancoderId) {}

}
