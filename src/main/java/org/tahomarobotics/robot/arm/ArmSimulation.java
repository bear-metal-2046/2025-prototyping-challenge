package org.tahomarobotics.robot.arm;

import static edu.wpi.first.units.Units.*;

import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.DiffMotorPositions;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.DiffMotorVelocities;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.DiffMotorVoltages;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.VirtualMotorPositions;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.VirtualMotorVelocities;
import org.tahomarobotics.robot.arm.ArmDifferentialTransform.VirtualMotorVoltages;
import org.tahomarobotics.robot.sim.AbstractSimulation;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * 
 */
public class ArmSimulation extends AbstractSimulation{

    private static final double ELBOW_GEARBOX_RATIO = 60d / 10d * 48d / 12d;
    private static final double WRIST_GEARBOX_RATIO = ELBOW_GEARBOX_RATIO * 45d / 15d;

    private static final double WRIST_MOI = 0.00156; // kg*m^2, estimated
    
    private static final double dT = Robot.defaultPeriodSecs;
    private static final double WRIST_ENCODER_GEARBOX_RATIO = 52d / 15d;

    private final TalonFXSimState topMotorSimState;
    private final TalonFXSimState bottomMotorSimState;


    private final SingleJointedArmSim elbowSim;
    private final DCMotorSim wristSim;

    // Holds the current velocities of the differential motors
    private DiffMotorVelocities diffMotorVelocities = new DiffMotorVelocities(
        RadiansPerSecond.of(0.0),
        RadiansPerSecond.of(0.0)
    );

    public ArmSimulation(TalonFXSimState topMotorSimState, TalonFXSimState bottomMotorSimState) {
        super("ArmSimulation");
        this.topMotorSimState = topMotorSimState;
        this.bottomMotorSimState = bottomMotorSimState;


        // Set motor orientations to match real robot (positive voltage moves arm up)
        topMotorSimState.Orientation = com.ctre.phoenix6.sim.ChassisReference.CounterClockwise_Positive;
        bottomMotorSimState.Orientation = com.ctre.phoenix6.sim.ChassisReference.Clockwise_Positive;



        DCMotor elbowMotor = DCMotor.getKrakenX60Foc(2).withReduction(ELBOW_GEARBOX_RATIO);

        elbowSim = new SingleJointedArmSim(
            elbowMotor,
            ELBOW_GEARBOX_RATIO,
            0.5, // 0.5 kg*m^2 moment of inertia
            5.0, // 5 kg counterweight at the end of the arm
            Math.toRadians(-100.0), // min angle
            Math.toRadians(100.0), // max angle
            true, // has gravity
            Math.toRadians(90.0) // initial angle
        );

        DCMotor wristMotor = DCMotor.getKrakenX60Foc(2);

        wristSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(wristMotor, WRIST_MOI, WRIST_GEARBOX_RATIO), 
          wristMotor);
    }

    @Override
    protected void simulationPeriodic() {

        // update simulations
        // -------------------------------------------------------------------
        
        // Update supply voltages
        double supplyVoltage = RobotController.getBatteryVoltage();
        topMotorSimState.setSupplyVoltage(supplyVoltage);
        bottomMotorSimState.setSupplyVoltage(supplyVoltage);
        
        // Get motor voltages
        DiffMotorVoltages diffMotorVoltages = new DiffMotorVoltages(
            topMotorSimState.getMotorVoltageMeasure(), 
            bottomMotorSimState.getMotorVoltageMeasure()
        );

        // Transform to virtual motor voltages (uses differential kinematics and previous velocities)
        VirtualMotorVoltages virtualMotorVoltages = ArmDifferentialTransform.transform(
            diffMotorVoltages,
            diffMotorVelocities
        );

        // Update elbow simulation
        elbowSim.setInput(virtualMotorVoltages.elbowMotorVoltage().in(Volts));
        elbowSim.update(dT);

        // update wrist simulation
        wristSim.setInput(virtualMotorVoltages.wristMotorVoltage().in(Volts));
        wristSim.update(dT);

        // update positions
        // -------------------------------------------------------------------
        
        // convert simulated joint positions to motor positions (multiply by gearbox ratio)
        VirtualMotorPositions virtualMotorPositions = new VirtualMotorPositions(
            Radians.of(elbowSim.getAngleRads() * ELBOW_GEARBOX_RATIO),
            Radians.of(wristSim.getAngularPositionRad() * WRIST_GEARBOX_RATIO)
        );

        // transform to differential motor positions
        DiffMotorPositions diffMotorPositions = ArmDifferentialTransform.transform(virtualMotorPositions);

        // update rotor positions in simulation states
        topMotorSimState.setRawRotorPosition(diffMotorPositions.topMotorPosition());
        bottomMotorSimState.setRawRotorPosition(diffMotorPositions.bottomMotorPosition());


        // update velocities
        // -------------------------------------------------------------------
        
        // Update motor velocities, joint velocities needs to be multiplied by gearbox ratio
        VirtualMotorVelocities virtualMotorVelocities = new VirtualMotorVelocities(
            RadiansPerSecond.of(elbowSim.getVelocityRadPerSec() * ELBOW_GEARBOX_RATIO), 
            RadiansPerSecond.of(wristSim.getAngularVelocityRadPerSec() * WRIST_GEARBOX_RATIO)
        );

        // transform back to differential motor velocities (save for next iteration)
        diffMotorVelocities = ArmDifferentialTransform.transform(virtualMotorVelocities);

        // update motor simulation states
        topMotorSimState.setRotorVelocity(diffMotorVelocities.topMotorAngularVelocity());
        bottomMotorSimState.setRotorVelocity(diffMotorVelocities.bottomMotorAngularVelocity());
        


    }
}
