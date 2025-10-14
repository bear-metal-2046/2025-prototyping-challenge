package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.tahomarobotics.robot.RobotMap;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for ArmSimulation
 *
 * Tests verify the ArmSimulation behavior using CTRE Phoenix 6 SimState API.
 * These tests create real hardware objects and manipulate their simulated state.
 */
@DisplayName("Arm Simulation Tests")
class ArmSimulationTest {

    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(2);
    private static final double ELBOW_GEARBOX_RATIO = 60.0 / 10.0 * 48.0 / 12.0; // 24:1
    private static final double WRIST_GEARBOX_RATIO = 45d / 15d;
    private static final double WRIST_ENCODER_GEARBOX_RATIO = 52d / 15d;

    @BeforeAll
    static void initializeHAL() {
        // HAL will be initialized per test
    }

    @AfterAll
    static void shutdownHAL() {
        // HAL will be shutdown per test
    }

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private CANcoder topEncoder;
    private CANcoder bottomEncoder;
    private CANcoder wristEncoder;
    private ArmSimulation armSimulation;

    private StatusSignal<Angle> topMotorPosition;
    private StatusSignal<AngularVelocity> topMotorVelocity;
    private StatusSignal<Angle> bottomMotorPosition;
    private StatusSignal<AngularVelocity> bottomMotorVelocity;
    private StatusSignal<Angle> topEncoderPosition;
    private StatusSignal<AngularVelocity> topEncoderVelocity;
    private StatusSignal<Angle> bottomEncoderPosition;
    private StatusSignal<AngularVelocity> bottomEncoderVelocity;
    private StatusSignal<Angle> wristEncoderPosition;
    private StatusSignal<AngularVelocity> wristEncoderVelocity;

    @BeforeEach
    void setup() {
        // Initialize HAL for WPILib simulation
        assert HAL.initialize(500, 0);

       
        SimHooks.stepTiming(0.020); // Initialize simulation state

        // Create hardware objects (work in simulation mode)
        topMotor = new TalonFX(10);
        topMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        //topMotor.stopMotor();
        topMotorPosition = topMotor.getPosition();
        topMotorVelocity = topMotor.getVelocity();
        
        bottomMotor = new TalonFX(11);
        bottomMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
        //bottomMotor.stopMotor();
        bottomMotorPosition = bottomMotor.getPosition();
        bottomMotorVelocity = bottomMotor.getVelocity();

        
        topEncoder = new CANcoder(12);
        topEncoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));
        topEncoderPosition = topEncoder.getPosition();
        topEncoderVelocity = topEncoder.getVelocity();

        bottomEncoder = new CANcoder(13);
        bottomEncoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)));
        bottomEncoderPosition = bottomEncoder.getPosition();
        bottomEncoderVelocity = bottomEncoder.getVelocity();

        wristEncoder = new CANcoder(14);
        wristEncoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));
        wristEncoderPosition = wristEncoder.getPosition();
        wristEncoderVelocity = wristEncoder.getVelocity();

        // Create simulation with hardware sim states
        armSimulation = new ArmSimulation(
            topMotor.getSimState(),
            bottomMotor.getSimState(),
            topEncoder.getSimState(),
            bottomEncoder.getSimState(),
            wristEncoder.getSimState()
        );

         DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        Timer.delay(0.100);

    }

    private void waitForAll() {
        BaseStatusSignal.waitForAll(0.10,
            topMotorPosition, topMotorVelocity,
            bottomMotorPosition, bottomMotorVelocity,
            topEncoderPosition, topEncoderVelocity,
            bottomEncoderPosition, bottomEncoderVelocity,
            wristEncoderPosition, wristEncoderVelocity);
    }

    @AfterEach
    void teardown() throws InterruptedException{
        // Close hardware objects to clean up resources
        if (topMotor != null) {
            topMotor.stopMotor();
            topMotor.close();
        }
        if (bottomMotor != null) {
            bottomMotor.stopMotor();
            bottomMotor.close();
        }
        if (topEncoder != null) {
            topEncoder.close();
        }
        if (bottomEncoder != null) {
            bottomEncoder.close();
        }
        if (wristEncoder != null) {
            wristEncoder.close();
        }

        RoboRioSim.resetData();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        // Shutdown HAL to reset simulation state
        HAL.shutdown();

        Thread.sleep(100);
    }

    @Test
    @DisplayName("Initial Conditions: Elbow at 90°, Wrist at 0°, Input shafts equal")
    void testInitialConditions() throws InterruptedException {
        // Constants from ArmSimulation
        

        // Run simulation for one cycle with default initialization (no control inputs)
        armSimulation.simulationPeriodic();
        waitForAll();

        // Differential mechanism should have equal shaft positions for 90° elbow, 0° wrist
        assertEquals(90.0, topEncoderPosition.getValue().in(Degrees), 1.0,
            "Top shaft should reflect 90° elbow angle");
        assertEquals(90.0, bottomEncoderPosition.getValue().in(Degrees), 1.0,
            "Bottom shaft should reflect 90° elbow angle");
        assertEquals(0.0, wristEncoderPosition.getValue().in(Degrees), 1.0,
            "Wrist shaft should reflect 0° wrist angle");

        // Verify motor-to-shaft gear ratio
        // Motors are positioned to reflect 90° elbow angle after simulation   
        assertEquals(90.0, topMotorPosition.getValue().div(ELBOW_GEARBOX_RATIO).in(Degrees), 0.01,
            "Top shaft encoder should reflect 24:1 gearbox reduction");
        assertEquals(90.0, bottomMotorPosition.getValue().div(ELBOW_GEARBOX_RATIO).in(Degrees), 0.01,
            "Bottom shaft encoder should reflect 24:1 gearbox reduction");
    }

    @Test
    @DisplayName("Differential Kinematics: Equal velocities → zero wrist velocity")
    void testEqualVelocities() throws InterruptedException {
        final double TEST_VOLTAGE = 12.0; // Volts - higher voltage for testing
        final int SIMULATION_CYCLES = 5; // Run for 0.1 seconds (5 cycles * 20ms)

        // Test: Equal motor voltages → only elbow moves (zero wrist velocity)
        topMotor.setControl(new VoltageOut(TEST_VOLTAGE));
        bottomMotor.setControl(new VoltageOut(TEST_VOLTAGE));

        double initialWristPosition = 0.0;
        for (int i = 0; i < SIMULATION_CYCLES; i++) {
            armSimulation.simulationPeriodic();
            waitForAll();
            if (i == 0) {
                initialWristPosition = wristEncoderPosition.getValueAsDouble();
            }
        }

        double finalWristPosition = wristEncoderPosition.getValueAsDouble();
        double wristChange = Math.abs(finalWristPosition - initialWristPosition);

        // With equal motor velocities, wrist should move very little (ideally zero)
        assertTrue(wristChange < 0.1,
            "Equal motor voltages should result in minimal wrist movement (actual change: " + wristChange + ")");

        // Verify that input shafts moved equally
        double topShaftPosition = topEncoder.getPosition().getValueAsDouble();
        double bottomShaftPosition = bottomEncoder.getPosition().getValueAsDouble();

        assertEquals(topShaftPosition, bottomShaftPosition, 0.1,
            "Equal motor voltages should keep shaft encoders equal");
    }

    @Test
    @DisplayName("Differential Kinematics: Opposite velocities → zero elbow velocity")
    void testOppositeVelocities() {
        final double TEST_VOLTAGE = 12.0; // Volts - higher voltage for testing
        final int SIMULATION_CYCLES = 5; // Run for 2 seconds (100 cycles * 20ms)

        // Run one cycle to set initial positions
        armSimulation.simulationPeriodic();
        waitForAll();

        // Get initial positions after setting to initial state
        double initialTopShaftPos = topEncoderPosition.getValueAsDouble();
        double initialBottomShaftPos = bottomEncoderPosition.getValueAsDouble();
        double initialWristPos = wristEncoderPosition.getValueAsDouble();

        // Verify initial conditions: shafts should be equal and properly initialized
        assertEquals(initialTopShaftPos, initialBottomShaftPos, 0.01,
            "Initial shaft positions should be equal for 90° elbow, 0° wrist");
        assertTrue(Math.abs(initialTopShaftPos) > 0.1,
            "Initial shaft positions should be non-zero (simulation should set initial positions)");

        // Test: Opposite motor voltages → only wrist moves (zero elbow velocity)
        topMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(TEST_VOLTAGE));
        bottomMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(-TEST_VOLTAGE));

        for (int i = 0; i < SIMULATION_CYCLES; i++) {
            armSimulation.simulationPeriodic();
            waitForAll();
        }

        double finalTopShaftPos = topEncoderPosition.getValueAsDouble();
        double finalBottomShaftPos = bottomEncoderPosition.getValueAsDouble();

        // Calculate average shaft position (represents elbow motion)
        double initialAvgShaftPos = (initialTopShaftPos + initialBottomShaftPos) / 2.0;
        double finalAvgShaftPos = (finalTopShaftPos + finalBottomShaftPos) / 2.0;
        double elbowChange = Math.abs(finalAvgShaftPos - initialAvgShaftPos);

        // Verify wrist did move
        double finalWristPos = wristEncoderPosition.getValueAsDouble();
        double wristMovement = Math.abs(finalWristPos - initialWristPos);
        assertTrue(wristMovement > 0.1,
            "Opposite motor voltages should cause wrist to move");

        // With opposite motor velocities, elbow should move much less than wrist
        assertTrue(elbowChange < 0.01,
            "Opposite motor voltages should result in elbow moving much less than wrist (elbow change: " + elbowChange + ", wrist change: " + wristMovement + ")");

        // Verify shaft encoders moved in opposite directions
        double topShaftChange = finalTopShaftPos - initialTopShaftPos;
        double bottomShaftChange = finalBottomShaftPos - initialBottomShaftPos;
        assertTrue(topShaftChange * bottomShaftChange < 0,
            "Opposite motor voltages should cause shafts to move in opposite directions");
    }

    @Test
    @DisplayName("Wrist Rotation: Voltage/Speed characteristics at multiple speeds")
    void testRotationVoltageSpeedCharacteristics() {
        
        // Test voltages to evaluate: 25%, 50%, 75%, and 100% of nominal voltage
        final double[] TEST_VOLTAGES = {3.0, 6.0, 9.0, 12.0}; // Volts
        final int WARMUP_CYCLES = 25; // 0.5 seconds to reach steady state

        // For continuous wrist rotation, we expect a roughly linear voltage/speed relationship
        // at steady state (after acceleration phase)

        for (double voltage : TEST_VOLTAGES) {
            
            // Run one cycle to set initial positions
            armSimulation.simulationPeriodic();
            waitForAll();

            // Apply opposite voltages to rotate wrist without moving elbow
            topMotor.setControl(new VoltageOut(voltage));
            bottomMotor.setControl(new VoltageOut(-voltage));

            // Warmup phase: allow motor to reach steady-state velocity
            for (int i = 0; i < WARMUP_CYCLES; i++) {
                armSimulation.simulationPeriodic();
                waitForAll();
            }
            
            double expectedElbowVelocity = Units.radiansToDegrees(MOTOR.getSpeed(0d, voltage)) / ELBOW_GEARBOX_RATIO;
            double expectedWristVelocity = expectedElbowVelocity / WRIST_GEARBOX_RATIO;

            
            // read encoder velocity
            double actualTopVelocity = topEncoderVelocity.getValue().in(DegreesPerSecond);
            double actualBottomVelocity = bottomEncoderVelocity.getValue().in(DegreesPerSecond);
            double actualWristVelocity = wristEncoderVelocity.getValue().in(DegreesPerSecond) / WRIST_ENCODER_GEARBOX_RATIO;

            // Verify velocities are within expected range
            assertTrue(Math.abs(actualWristVelocity - expectedWristVelocity) < 1.0,
                "Wrist motor velocity deviated from expected (actual: " + actualWristVelocity + ", expected: " + expectedWristVelocity + ")");
            assertTrue(Math.abs(actualTopVelocity - expectedElbowVelocity) < 1.0,
                "Top motor velocity deviated from expected (actual: " + actualTopVelocity + ", expected: " + expectedElbowVelocity + ")");
            assertTrue(Math.abs(actualBottomVelocity - expectedElbowVelocity) < 1.0,
                "Bottom motor velocity deviated from expected (actual: " + actualBottomVelocity + ", expected: " + expectedElbowVelocity + ")");
        }
    }



    /*
     * Recommended test structure (once hardware mocking is set up):
     *
     * 1. Initial Conditions
     *    - Elbow starts at 90°
     *    - Wrist starts at 0°
     *    - Input shafts equal at start
     *    - Motor/shaft gear ratios verified
     *
     * 2. Differential Kinematics
     *    - Equal motor velocities → zero wrist velocity
     *    - Opposite velocities → zero elbow velocity
     *    - Bevel gear 1.5× amplification
     *    - Shaft encoders = motor ÷ 24
     *    - Wrist encoder reflects 52:15 belt ratio
     *
     * 3. Pure Motion Modes
     *    - Equal voltages → only elbow moves
     *    - Opposite voltages → only wrist moves
     *
     * 4. Physical Constraints
     *    - Hard stops at ±100° for elbow
     *    - Wrist rotates continuously
     *    - Gravity pulls elbow down
     *
     * 5. Energy and Dynamics
     *    - Velocity decay with zero voltage
     *    - Positive/negative voltage direction
     *    - Higher voltage → faster motion
     *
     * 6. Edge Cases
     *    - Voltage reversals
     *    - Rapid voltage changes
     *    - Long simulation stability
     */
}
