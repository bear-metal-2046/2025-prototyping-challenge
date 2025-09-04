// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.util.SubsystemIF;


public class OI extends SubsystemIF
{
    private static final OI INSTANCE = new OI();

    public static OI getInstance() {
        return INSTANCE;
    }

    private final CommandXboxController driverController =
            new CommandXboxController(0);
    

    private OI()
    {
        // Configure the trigger bindings
        configureBindings();
    }


    private void configureBindings()
    {

    }
}
