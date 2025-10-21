package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import static org.tahomarobotics.robot.RobotMap.CANBUS_NAME;

public class ChassisSubsystem extends AbstractSubsystem {



    //swerve Modules
    private List<SwerveModule> modules;
    //Gyro
    private Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, CANBUS_NAME);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity>yawRate = pigeon.getAngularVelocityZWorld();


    private final LoggedStatusSignal[] statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Yaw", yaw),
            new LoggedStatusSignal("Yaw Velocity", yawRate)
    };


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
        Logger.recordMetadata("ChassisSubsystem creation", "creating new ChassisSubsystem");

    }

    @Override
    public void subsystemPeriodic() {

         Logger.recordOutput("Yaw", yaw.getValue());
         Logger.recordOutput("Yaw Rate", yawRate.getValue());

    }
}
