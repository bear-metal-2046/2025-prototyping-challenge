package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tinylog.Logger;

import java.util.List;

import static org.tahomarobotics.robot.RobotMap.CANBUS_NAME;

public class ChassisSubsystem extends AbstractSubsystem {


    //swerve Modules
    private List<SwerveModule> modules;
    //Gyro
    private Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, CANBUS_NAME);


    //Constructor

    ChassisSubsystem(List modules, Pigeon2 pigeon) {
        this.modules = modules;
        this.pigeon = pigeon;
    }

    //Constructor
    ChassisSubsystem() {

        modules = List.of(

                new SwerveModule(RobotMap.FRONT_LEFT_MODULE),
                new SwerveModule(RobotMap.FRONT_RIGHT_MODULE),
                new SwerveModule(RobotMap.BACK_LEFT_MODULE),
                new SwerveModule(RobotMap.BACK_RIGHT_MODULE)


        );
        Logger.info("Creating an instance of ChassisSubsystem...");

    }

    @Override
    public void subsystemPeriodic() {

    }
}
