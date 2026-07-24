package frc.robot.util.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.crystalcaverns.Colors.CrystalColor;

public class CrystalOnFly extends GamePieceProjectile {
    public final CrystalColor color;

    public CrystalOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle,
            CrystalColor color) {
        super(
                CrystalOnField.CRYSTAL_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        super.withTouchGroundHeight(0.6);
        super.enableBecomesGamePieceOnFieldAfterTouchGround();

        this.color = color;
    }

    @Override
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!becomesGamePieceOnGroundAfterTouchGround) return;
        simulatedArena.addGamePiece(new CrystalOnField(
            () -> Math.max(
                        super.info.gamePieceHeight().in(Meters) / 2,
                        super.getPositionAtTime(launchedTimer.get()).getZ()),
            new Pose2d(super.getPositionAtTime(launchedTimer.get()).toTranslation2d(), new Rotation2d()),
            initialLaunchingVelocityMPS,
            this.color
        ));
    }
}
