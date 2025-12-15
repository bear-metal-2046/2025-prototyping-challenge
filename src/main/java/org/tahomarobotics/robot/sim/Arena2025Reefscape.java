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

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.tahomarobotics.robot.util.ExecutionTimeLogger;

import java.util.Collections;
import java.util.Set;

/**
 * Extended the default Arena to include some needed functionality
 */
public class Arena2025Reefscape extends org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape {

    static {
        org.ironmaple.simulation.SimulatedArena.overrideInstance(new Arena2025Reefscape());
    }

    // need access to dyn4j physics' engine to run RayCasting
    public World<Body> getWorld() {
        return physicsWorld;
    }

    private SwerveDriveSimulation swerveDriveSimulation = null;

    // used by chassis
    public void setSwerveDriveSimulation(SwerveDriveSimulation swerveDriveSimulation) {
        addDriveTrainSimulation(swerveDriveSimulation);
        this.swerveDriveSimulation = swerveDriveSimulation;
    }

    // used by vision simulation to estimate real distances to targets
    public SwerveDriveSimulation getSwerveDriveSimulation() {
        assert swerveDriveSimulation != null;
        return swerveDriveSimulation;
    }

    /**
     * Returns the singleton instance of the class
     */
    public static Arena2025Reefscape getInstance() {
        var instance = SimulatedArena.getInstance();
        assert(instance instanceof Arena2025Reefscape);
        return (Arena2025Reefscape) instance;
    }

    public Set<GamePiece> getGamePieceOnField() {
        return Collections.unmodifiableSet(gamePieces);
    }

    public synchronized void loggedPeriodic() {
        ExecutionTimeLogger.logExecutionTime("Simulation/Arena", this::simulationPeriodic);
    }
}
