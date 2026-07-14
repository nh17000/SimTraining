package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.crystalcaverns.FieldConstants;

import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoAlign {
    private Pose2d targetPose = Pose2d.kZero;

    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    @Getter
    @AutoLogOutput
    private boolean isForwards = true;

    private Supplier<Pose2d> robotSupplier;

    public AutoAlign(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command caveAlignUpper(Drive drive) {
        return caveAlign(drive, false);
    }

    public Command caveAlignLower(Drive drive) {
        return caveAlign(drive, true);
    }

    private Command caveAlign(Drive drive, boolean isUpper) {
        return new DriveToPose(drive, () -> findCaveTargetPose(robotSupplier.get(), isUpper), robotSupplier);
    }

    private Pose2d findCaveTargetPose(Pose2d currentPose, boolean isUpper) {
        Pose2d tagPose = currentPose.nearest(FieldConstants.ALIGN_POSES);

        Rotation2d backwardsRotation = tagPose.getRotation();
        Rotation2d forwardsRotation = backwardsRotation.plus(Rotation2d.k180deg);

        Rotation2d currRot2d = currentPose.getRotation();

        double fwdError = forwardsRotation.minus(currRot2d).getDegrees();
        double bwdError = backwardsRotation.minus(currRot2d).getDegrees();

        isForwards = Math.abs(fwdError) < Math.abs(bwdError);
        
        Translation2d offset = Translation2d.kZero;
        if (isUpper) {
            boolean alignLeft = (tagPose.getRotation().getDegrees() + 180) % 90 > 45;
            offset = alignLeft ? AlignConstants.LEFT_BRANCH_OFFSET : AlignConstants.RIGHT_BRANCH_OFFSET;
        }

        return targetPose =
                tagPose.transformBy(new Transform2d(offset, isForwards ? Rotation2d.k180deg : Rotation2d.kZero));
    }

    @AutoLogOutput
    public boolean isAligned() {
        Transform2d error = targetPose.minus(robotSupplier.get());

        return error.getTranslation().getNorm() < AlignConstants.ALIGN_TRANSLATION_TOLERANCE
                && error.getRotation().getRadians() < AlignConstants.ALIGN_ROT_TOLERANCE;
    }

    @AutoLogOutput
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
    }
}
