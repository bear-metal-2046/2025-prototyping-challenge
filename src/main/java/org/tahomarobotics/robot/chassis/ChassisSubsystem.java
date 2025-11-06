package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON, CANBUS_NAME);
    private Rotation2d heading = new Rotation2d();
    private Rotation2d rotation2d = new Rotation2d();



    private final StatusSignal<Angle> yaw = gyro.getYaw();
    private final StatusSignal<AngularVelocity>yawRate = gyro.getAngularVelocityZWorld();


    private final LoggedStatusSignal[] statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Yaw", yaw),
            new LoggedStatusSignal("Yaw Velocity", yawRate)
    };


    //Constructor

     ChassisSubsystem(List modules, Pigeon2 pigeon) {
        this.modules = modules;
        this.gyro = pigeon;
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

    public Rotation2d getHeading() {
        double yawDegrees = yaw.getValue().in(edu.wpi.first.units.Units.Degrees); // Convert Angle to degrees
        heading = Rotation2d.fromDegrees(yawDegrees);
        return heading;
    }

    public Rotation2d getRotation2d() {
         rotation2d = gyro.getRotation2d();
        return rotation2d;
    }

    @Override
    public void subsystemPeriodic() {

         Logger.recordOutput("Chassis/Yaw", yaw.getValue());
         Logger.recordOutput("Chassis/Yaw Rate", yawRate.getValue());
         Logger.recordOutput("Chassis/Heading", heading.getDegrees());

    }
}
