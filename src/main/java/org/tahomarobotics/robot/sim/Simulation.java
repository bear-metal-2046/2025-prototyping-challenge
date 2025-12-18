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

package org.tahomarobotics.robot.sim;

import org.ironmaple.simulation.SimulatedArena;
import org.tahomarobotics.robot.chassis.ChassisSimulation;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;

public class Simulation {

    private static final Time SIM_LOOP_PERIOD = Seconds.of(0.002);
    private final Notifier simNotifier;

    private final Arena2025Reefscape arena;
    private final ChassisSimulation chassisSimulation;

    public Simulation(
            Arena2025Reefscape arena,
            ChassisSimulation chassisSimulation) {

        this.arena = arena;
        this.chassisSimulation = chassisSimulation;

        SimulatedArena.overrideSimulationTimings(SIM_LOOP_PERIOD, 1);

        simNotifier = new Notifier(this::update);
        
    }

    public void init() {
        simNotifier.startPeriodic(SIM_LOOP_PERIOD.in(Seconds));
    }

    public void resetFieldForAuto() {
        arena.resetFieldForAuto();
    }

    /**
     * High rate simulation update loop.
     */
    private void update() {
        arena.simulationPeriodic();
        chassisSimulation.update();
    }

    /**
     * Called periodically during simulation mode.
     */
    public void simulationPeriodic() {
        chassisSimulation.simulationPeriodic();
    }

}