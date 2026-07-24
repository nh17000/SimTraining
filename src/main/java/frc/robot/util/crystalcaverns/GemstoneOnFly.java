package frc.robot.util.crystalcaverns;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.crystalcaverns.Colors.GemstoneColor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class GemstoneOnFly extends GamePieceProjectile {
    public final GemstoneColor color;

    public GemstoneOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle,
            GemstoneColor color) {
        super(
                GemstoneOnField.GEMSTONE_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        // cone's origin is at bottom
        // might clip through ground if rotated...
        super.withTouchGroundHeight(Units.inchesToMeters(0.5)); 
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        this.color = color;
    }

    @Override
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!becomesGamePieceOnGroundAfterTouchGround) return;
        simulatedArena.addGamePiece(new GemstoneOnField(
            () -> Math.max(
                        0,
                        super.getPositionAtTime(launchedTimer.get()).getZ()),
            new Pose2d(super.getPositionAtTime(launchedTimer.get()).toTranslation2d(), new Rotation2d()),
            initialLaunchingVelocityMPS,
            this.color
        ));
    }
}
