package frc.robot.util.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class GemstoneOnField extends GamePieceOnFieldSimulation {
    // TODO: triangle if horizontal or rectangle if upright?
    public static final GamePieceInfo GEMSTONE_INFO =
            new GamePieceInfo("Gemstone", new Rectangle(0.3, 0.11), Meters.of(0.11), Kilograms.of(0.3), 2.8, 4, 0.3);

    public GemstoneOnField(Translation2d initialPosition) {
        super(GEMSTONE_INFO, new Pose2d(initialPosition, new Rotation2d()));
    }
}
