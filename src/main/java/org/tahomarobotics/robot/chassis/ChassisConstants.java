package org.tahomarobotics.robot.chassis;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

// Note that constants referring to x and y mean forward on the robot is positive x, and right on the robot is positive y.

public class ChassisConstants {

    private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor STEER_MOTOR = DCMotor.getKrakenX60(1);

    /** Approximate width of the robot with bumpers. */
    public static final Distance BUMPER_WIDTH_X = Inches.of(34);
    public static final Distance BUMPER_WIDTH_Y = Inches.of(34);

    /** Y distance between wheel centers */
    private static final Distance TRACK_DISTANCE_X = Inches.of(20.75);
    /** X distance between wheel centers */
    private static final Distance TRACK_DISTANCE_Y = Inches.of(20.75);

    /** Approximate mass of the robot in <strong>kilograms</strong> */
    private static final Mass BATTERY_MASS = Pounds.of(13.6);
    private static final Mass BUMPER_MASS = Pounds.of(16.0);
    private static final Mass ROBOT_MASS = Pounds.of(119.0);
    public static final Mass TOTAL_MASS = ROBOT_MASS.plus(BATTERY_MASS).plus(BUMPER_MASS);

    // Swerve module characteristics for simulation
    public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(0.05);
    public static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.3);
    public static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.5);

    public static Supplier<SwerveModuleSimulation> swerveModuleSimulation(DriveGearing driveGearing, SteerGearing steerGearing) {
        return new SwerveModuleSimulationConfig(
                DRIVE_MOTOR, 
                STEER_MOTOR, 
                driveGearing.driveReduction, 
                steerGearing.steerReduction, 
                DRIVE_FRICTION_VOLTAGE, 
                STEER_FRICTION_VOLTAGE, 
                Inches.of(2), 
                MOTOR_MOI, 
                1.2);
    }

    public static DriveTrainSimulationConfig swerveDriveConfig() {
        Supplier<SwerveModuleSimulation> frontLeftModuleSim = swerveModuleSimulation(DriveGearing.L2_PLUS, SteerGearing.MK4I);
        Supplier<SwerveModuleSimulation> frontRightModuleSim = swerveModuleSimulation(DriveGearing.L2_PLUS, SteerGearing.MK4I);
        Supplier<SwerveModuleSimulation> backLeftModuleSim = swerveModuleSimulation(DriveGearing.L2_PLUS, SteerGearing.MK4I);
        Supplier<SwerveModuleSimulation> backRightModuleSim = swerveModuleSimulation(DriveGearing.L2_PLUS, SteerGearing.MK4I);
        return  new DriveTrainSimulationConfig(
            TOTAL_MASS,
            BUMPER_WIDTH_X,
            BUMPER_WIDTH_Y,
            TRACK_DISTANCE_X,
            TRACK_DISTANCE_Y,
            COTS.ofPigeon2(),
            frontLeftModuleSim,
            frontRightModuleSim,
            backLeftModuleSim,
            backRightModuleSim
            );
    }


    public enum DriveGearing {
        // L ratios as Mk4, R ratios are Mk5
        L1_PLUS((1.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)),
        L2_PLUS((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)),
        L3_PLUS((16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)),
        L4_PLUS((16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0)),
        R1((12.0 / 54.0) * (32 / 25) * (15 / 30)),
        R2((14 / 54) * (32 / 25) * (15 / 30)),
        R3((16 / 54) * (32 / 25) * (15 / 30));

        final double driveReduction;

        DriveGearing(double driveReduction) {
            this.driveReduction = driveReduction;
        }
    }

    public enum SteerGearing {
        MK4I(50.0 / 14.0 * 60.0 / 10.0),
        MK4N(50.0 / 16.0 * 60.0 / 10.0),
        // TODO update MK5N ratios to show each gear instead of final ratio
        MK5N(287.0 / 11.0);

        final double steerReduction;

        SteerGearing(double steerReduction) {
            this.steerReduction = steerReduction;
        }
    }
}
