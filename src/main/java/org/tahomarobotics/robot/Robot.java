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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.tahomarobotics.robot.util.CommandLogger;
import org.tahomarobotics.robot.util.WatchDog;

import static edu.wpi.first.units.Units.Inches;

public class Robot extends LoggedRobot {

    public static final Pose2d STARTING_ROBOT_POSE = new Pose2d(3,3, new Rotation2d());

    private final RobotContainer robotContainer;
    private final OI oi;

    private Robot() {
        this(new RobotContainer());
    }

    Robot(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        oi = new OI(robotContainer);

        if (Robot.isSimulation()) {
            WatchDog.disableWatchdog(this);
            WatchDog.disableWatchdog(CommandScheduler.getInstance());
        }

        // initialize AdvantageScope publishing
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        // logs the duration of commands that finish or get interrupted
        CommandLogger.install();

    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        robotContainer.elevator.updateTelemetry();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
        // no need for this other than removing console printouts
    }

    @Override
    public void disabledInit() {
        
    }

    @Override
    public void teleopPeriodic() {


        // no need for this other than removing console printouts
    }

    @Override
    public void disabledPeriodic() {
        // no need for this other than removing console printouts
    }

    @Override
    public void simulationInit() {
        robotContainer.simulation.resetFieldForAuto();
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.simulation.loggedPeriodic();

       
    }

    @Override
    public void close() {
        super.close();
        robotContainer.close();
    }

    public static void main(String... args) {
        // start this class or restart if something bad happened
        RobotBase.startRobot(Robot::new);
    }
}