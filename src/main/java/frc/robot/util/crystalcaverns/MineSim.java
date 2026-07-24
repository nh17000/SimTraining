package frc.robot.util.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

// Crystal Station
public class MineSim {
    public static double lastDropTime = 0;
    public static boolean lastAddRandomnessCheck = false;

    public static final double DROP_COOLDOWN_SECONDS = 5;

    public static Pose2d getMine() {
        return RobotContainer.isRedAlliance() ? FieldConstants.RED_MINE : FieldConstants.BLUE_MINE;
    }

    public static void drop(boolean addRandomness) {
        if (!canDrop() && lastAddRandomnessCheck == addRandomness) {
            return;
        }

        lastAddRandomnessCheck = addRandomness;        
        Pose2d minePose = getMine();        
        GamePieceProjectile gamePiece;

        if (addRandomness) {
            gamePiece = new CrystalOnFly(
                    minePose.getTranslation(),
                    new Translation2d(0, Math.random() * 1.5 - 0.75),
                    new ChassisSpeeds(),
                    minePose.getRotation().plus(Rotation2d.fromDegrees(Math.random() * 30 - 15)),
                    Meters.of(1),
                    MetersPerSecond.of(2.5 + Math.random() * 5),
                    Degrees.of(-55 + Math.random() * 30),
                    Colors.getRandomCrystalColor());
        } else {
            gamePiece = new CrystalOnFly(
                    minePose.getTranslation(),
                    Translation2d.kZero,
                    new ChassisSpeeds(),
                    minePose.getRotation(),
                    Meters.of(1),
                    MetersPerSecond.of(5),
                    Degrees.of(-55),
                    Colors.getRandomCrystalColor());
        }

        SimulatedArena.getInstance().addGamePieceProjectile(gamePiece);
        lastDropTime = Timer.getFPGATimestamp();
    }

    public static boolean canDrop() {
        return DriverStation.isEnabled()
                && Timer.getFPGATimestamp() - lastDropTime > DROP_COOLDOWN_SECONDS;
    }
}
