// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.List;


public class Robot extends TimedRobot
{
    List<SubsystemIF> subsystems = List.of(
            OI.getInstance()
    );

    public Robot()
    {
        for (SubsystemIF subsystem : subsystems) {
            CommandScheduler.getInstance().registerSubsystem(subsystem);
            subsystem.initialize();
        }
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        for (SubsystemIF subsystem : subsystems) {
            subsystem.onDisabledInit();
        }
    }
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    /** This autonomous runs the autonomous command selected by your {@link OI} class. */
    @Override
    public void autonomousInit()
    {
        for (SubsystemIF subsystem : subsystems) {
            subsystem.onAutonomousInit();
        }
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit()
    {
        for (SubsystemIF subsystem : subsystems) {
            subsystem.onTeleopInit();
        }
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void testInit()
    {
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    
    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        for (SubsystemIF subsystem : subsystems) {
            subsystem.onSimulationInit();
        }
    }
    
    
    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
