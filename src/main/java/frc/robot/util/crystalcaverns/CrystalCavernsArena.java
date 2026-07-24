package frc.robot.util.crystalcaverns;

import static frc.robot.util.crystalcaverns.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.crystalcaverns.Colors.CrystalColor;
import frc.robot.util.crystalcaverns.Colors.GemstoneColor;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.littletonrobotics.junction.Logger;

public class CrystalCavernsArena extends SimulatedArena {
    public static final class CrystalFieldObstaclesMap extends FieldMap {
        public CrystalFieldObstaclesMap() {
            super();

            super.addBorderLine(RED_MINE_CORNER_A, BLUE_MINE_CORNER_A);
            super.addBorderLine(BLUE_MINE_CORNER_A, BLUE_MINE_CORNER_B);
            super.addBorderLine(BLUE_MINE_CORNER_B, new Translation2d(FIELD_LENGTH, FIELD_WIDTH));
            super.addBorderLine(new Translation2d(FIELD_LENGTH, FIELD_WIDTH), new Translation2d(0, FIELD_WIDTH));
            super.addBorderLine(new Translation2d(0, FIELD_WIDTH), RED_MINE_CORNER_B);
            super.addBorderLine(RED_MINE_CORNER_B, RED_MINE_CORNER_A);

            // caves
            for (int i = 0; i < 8; i++) {
                super.addBorderLine(BLUE_CAVE_VORTICES[i], (BLUE_CAVE_VORTICES[(i + 1) % 8]));
                super.addBorderLine(RED_CAVE_VORTICES[i], (RED_CAVE_VORTICES[(i + 1) % 8]));
            }

            // cave exit trusses  
            for (int i = 0; i < 4; i++) {
                super.addBorderLine(BLUE_CAVE_EXIT_TRUSS_1_CORNERS[i], (BLUE_CAVE_EXIT_TRUSS_1_CORNERS[(i + 1) % 4]));
                super.addBorderLine(BLUE_CAVE_EXIT_TRUSS_2_CORNERS[i], (BLUE_CAVE_EXIT_TRUSS_2_CORNERS[(i + 1) % 4]));
                super.addBorderLine(RED_CAVE_EXIT_TRUSS_1_CORNERS[i], (RED_CAVE_EXIT_TRUSS_1_CORNERS[(i + 1) % 4]));
                super.addBorderLine(RED_CAVE_EXIT_TRUSS_2_CORNERS[i], (RED_CAVE_EXIT_TRUSS_2_CORNERS[(i + 1) % 4]));
            }

            // jewelry
            for (int i = 0; i < 6; i++) {
                super.addBorderLine(JEWELRY_VORTICES[i], (JEWELRY_VORTICES[(i + 1) % 6]));
            }
        }
    }

    public final CaveSim blueCaveSim;
    public final CaveSim redCaveSim;
    private final GemstoneHandler gemstoneHandler;

    public CrystalCavernsArena() {
        super(new CrystalFieldObstaclesMap());

        blueCaveSim = new CaveSim(this, true);
        redCaveSim = new CaveSim(this, false);
        gemstoneHandler = GemstoneHandler.getInstance();

        super.addCustomSimulation(blueCaveSim);
        super.addCustomSimulation(redCaveSim);
    }

    @Override
    protected void simulationSubTick(int subTickNum) {
        super.simulationSubTick(subTickNum);
        gemstoneHandler.periodic();
    }

    @Override
    public void placeGamePiecesOnField() {
        for (int i = 0; i < CRYSTAL_STARTING_POSITIONS.length; i++) {
            super.addGamePiece(new CrystalOnField(
                CRYSTAL_STARTING_POSITIONS[i].getTranslation(), CRYSTAL_STARTING_COLORS[i]));
        }
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        if (type.equals("Crystal")) {
            redCaveSim.draw(poses);
            blueCaveSim.draw(poses);
        } else if (type.equals("Gemstone")) {
            gemstoneHandler.draw(poses);
        }

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        redCaveSim.clearCave();
        blueCaveSim.clearCave();
        gemstoneHandler.reset();
    }

    public void logGamePieces(Optional<Pose3d> heldCrystalPose, CrystalColor heldCrystalColor) {
        List<List<Pose3d>> crystalPoses = new ArrayList<List<Pose3d>>();
        for (int i = 0; i < CrystalColor.values().length; i++) {
            crystalPoses.add(new ArrayList<Pose3d>());
        }
        
        List<GamePiece> crystals = getGamePiecesByType("Crystal");
        for (GamePiece gp : crystals) {
            if (gp instanceof CrystalOnField crystal) {
                int colorIdx = crystal.color.index;
                crystalPoses.get(colorIdx).add(crystal.getPose3d());
            } else if (gp instanceof CrystalOnFly crystal) {
                int colorIdx = crystal.color.index;
                crystalPoses.get(colorIdx).add(crystal.getPose3d());
            }
        }

        int idx = 0;
        for (CrystalColor c : CrystalColor.values()) {
            List<Pose3d> poses = crystalPoses.get(idx++);

            redCaveSim.drawByColor(poses, c);
            blueCaveSim.drawByColor(poses, c);

            if (heldCrystalPose.isPresent() && heldCrystalColor == c) {
                poses.add(heldCrystalPose.get());
            }

            Logger.recordOutput(
                "FieldSimulation/" + c.toString() + " Crystals", 
                poses.toArray(Pose3d[]::new));
        }

        List<GamePiece> gemstones = getGamePiecesByType("Gemstone");
        List<Pose3d> redGemstones = new ArrayList<>();
        List<Pose3d> blueGemstones = new ArrayList<>();

        for (GamePiece gp : gemstones) {
            if (gp instanceof GemstoneOnField gemstone) {
                switch (gemstone.color) {
                    case Red: redGemstones.add(gp.getPose3d()); break;
                    case Blue: blueGemstones.add(gp.getPose3d()); break;
                }
            } else if (gp instanceof GemstoneOnFly gemstone) {
                switch (gemstone.color) {
                    case Red: redGemstones.add(gp.getPose3d()); break;
                    case Blue: blueGemstones.add(gp.getPose3d()); break;
                }
            }
        }

        gemstoneHandler.drawByColor(redGemstones, GemstoneColor.Red);
        gemstoneHandler.drawByColor(blueGemstones, GemstoneColor.Blue);
        
        Logger.recordOutput("FieldSimulation/Red Gemstones", redGemstones.toArray(Pose3d[]::new));
        Logger.recordOutput("FieldSimulation/Blue Gemstones", blueGemstones.toArray(Pose3d[]::new));
    }
}
