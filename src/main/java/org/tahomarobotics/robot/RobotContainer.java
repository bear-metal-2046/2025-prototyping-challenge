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

import org.tahomarobotics.robot.vision.Vision;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.sim.Arena2025Reefscape;
import org.tahomarobotics.robot.sim.Simulation;


/**
 * The RobotContainer declares the robot's structure: subsystems, commands and simulations.
 * All the robot's components and their relationships are declared here rather than scattered
 * throughout the codebase.
 */
public class RobotContainer implements AutoCloseable {

    // must be created first here this needs to override the instance provided
    // by since org.ironmaple.simulation.SimulatedArena.getInstance() is used
    // some of MapleSim
    private final Arena2025Reefscape arena = org.tahomarobotics.robot.sim.Arena2025Reefscape.getInstance();

    public final Chassis chassis;
    public final Vision vision;
    public final OI oi;
    public final Simulation simulation;


    public RobotContainer() {
        chassis = new Chassis();
        vision = new Vision();
        oi = new OI(this);

        simulation = Robot.isSimulation() ? 
        new Simulation(
                arena,
                chassis.getSimulation()
        ) : null;
        
    }

    @Override
    public void close() {
    }
}