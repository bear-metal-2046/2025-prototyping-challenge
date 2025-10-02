package org.tahomarobotics.Chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.persistent.CalibrationData;

import java.util.List;

public class ChassisSubsystem extends SubsystemBase {
    protected static final ChassisSubsystem INSTANCE = new ChassisSubsystem();


    public static final String CANBUS_NAME = "Canivore"; /* should be in RobotConfiguration */
    //swerve Modules
    private final List<SwerveModule> modules;
    //Gyro
    private final Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, CANBUS_NAME);

    //State
    private final CalibrationData<Double[]> swerveCalibration;
    //Constructor
    private ChassisSubsystem() {


        swerveCalibration = new CalibrationData<>("Swerve Calibration", new Double[]{0d, 0d, 0d, 0d});


        Double[] angularOffsets = swerveCalibration.get();
        modules = List.of(
                new SwerveModule(RobotMap.FRONT_LEFT_MODULE, angularOffsets[0]),
                new SwerveModule(RobotMap.FRONT_RIGHT_MODULE, angularOffsets[1]),
                new SwerveModule(RobotMap.BACK_LEFT_MODULE, angularOffsets[2]),
                new SwerveModule(RobotMap.BACK_RIGHT_MODULE, angularOffsets[3])
        );
    }

}
