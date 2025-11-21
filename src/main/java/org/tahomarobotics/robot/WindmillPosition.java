package org.tahomarobotics.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public record WindmillPosition(Angle armAngle, Distance elevatorHeight) {

    public enum TeamPositions {
        TEAM_POSITIONS(
                // these are placeholder values and should be replaced with actual team-specific positions
                new WindmillPosition(Degrees.of(10), Meters.of(10)), // low
                new WindmillPosition(Degrees.of(10), Meters.of(10)), // mid
                new WindmillPosition(Degrees.of(10), Meters.of(10)), // high
                new WindmillPosition(Degrees.of(10), Meters.of(10)) // stow
        );

        private final WindmillPosition low;
        private final WindmillPosition mid;
        private final WindmillPosition high;
        private final WindmillPosition stow;

        TeamPositions(WindmillPosition low, WindmillPosition mid, WindmillPosition high, WindmillPosition stow) {
            this.low = low;
            this.mid = mid;
            this.high = high;
            this.stow = stow;
        }

        public WindmillPosition getLow() {
            return low;
        }

        public WindmillPosition getMid() {
            return mid;
        }
        public WindmillPosition getHigh() {
            return high;
        }

        public WindmillPosition getStow() {
            return stow;
        }
    }

}
