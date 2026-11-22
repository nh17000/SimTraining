package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class RobotVisualizer {
//     @AutoLogOutput(key = "RobotVisualizer/2D Visualizer")
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));

    private final LoggedMechanismRoot2d pivotRoot = mech2d.getRoot(
            "Pivot Root", -VisualizerConstants.STAGE0_ZERO.getX(), VisualizerConstants.STAGE0_ZERO.getZ());
    private final LoggedMechanismLigament2d arm = pivotRoot.append(new LoggedMechanismLigament2d(
            "Arm",
            ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH,
            ArmConstants.PIVOT_MIN_ANGLE,
            10,
            new Color8Bit(Color.kDarkSeaGreen)));
    private final LoggedMechanismLigament2d wrist1 = arm.append(new LoggedMechanismLigament2d(
            "Wrist",
            ArmConstants.WRIST_LENGTH,
            ArmConstants.WRIST_STARTING_ANGLE,
            4,
            new Color8Bit(Color.kLightSeaGreen)));
    private final LoggedMechanismLigament2d wrist2 = arm.append(new LoggedMechanismLigament2d(
            "Wrist2",
            ArmConstants.WRIST_LENGTH * 1.76,
            ArmConstants.WRIST_STARTING_ANGLE + 54.14,
            4,
            new Color8Bit(Color.kMediumSeaGreen)));

    private final LoggedTunableNumber x = new LoggedTunableNumber("RV/x", 0.065);
    private final LoggedTunableNumber y = new LoggedTunableNumber("RV/y", 0.0);
    private final LoggedTunableNumber z = new LoggedTunableNumber("RV/z", 0.225);
    private final LoggedTunableNumber roll = new LoggedTunableNumber("RV/r", 0.0);
    private final LoggedTunableNumber pitch = new LoggedTunableNumber("RV/p", 0.0);
    private final LoggedTunableNumber yaw = new LoggedTunableNumber("RV/yaw", 90.0);
    private final LoggedTunableNumber r2 = new LoggedTunableNumber("RV/r2", 0.0);
    private final LoggedTunableNumber p2 = new LoggedTunableNumber("RV/p2", 0.0);
    private final LoggedTunableNumber yaw2 = new LoggedTunableNumber("RV/yaw2", 0.0);

    private Supplier<ArmPosition> armPosSupplier;

    public RobotVisualizer(Supplier<ArmPosition> armPositionSupplier) {
        this.armPosSupplier = armPositionSupplier;
    }

    public void periodic() {
        // ArmPosition armPos = armPosSupplier.get();
        // double armAngle = armPos.getPivotRads();
        // double extensionLength = armPos.getExtensionMeters();
        // double wristAngle = armPos.getWristRads();

        // // Mechanism2d
        // arm.setAngle(Units.radiansToDegrees(armAngle));
        // arm.setLength(extensionLength + ArmConstants.ARM_SHOULDER_TO_WRIST_LENGTH);
        // wrist1.setAngle(Units.radiansToDegrees(wristAngle));
        // wrist2.setAngle(Units.radiansToDegrees(wristAngle) + 54.14);

        // // 3D Components
        // Logger.recordOutput("RobotVisualizer/Arm Components", armPos.getComponentTransforms());

        Logger.recordOutput("RobotVisualizer/Origin", Pose3d.kZero);

        // * 0.5 - Math.PI * 0.5;

        double intakeAngle = Math.sin(RobotController.getFPGATime() / 1e6) * Math.PI; 

        Transform3d intake = new Transform3d(
                new Translation3d(0.3302, 0.0, 0.17145),
                new Rotation3d(0, intakeAngle, 0));

        Logger.recordOutput("RobotVisualizer/intake", intake);
                
        // coral
        Transform3d coral = intake.plus(new Transform3d(
                    new Translation3d(x.get(), y.get(), z.get())
                            .rotateBy(new Rotation3d(
                                Units.degreesToRadians(r2.get()), 
                                Units.degreesToRadians(p2.get()), 
                                Units.degreesToRadians(yaw2.get()))),
                    new Rotation3d(
                        Units.degreesToRadians(roll.get()), 
                        Units.degreesToRadians(pitch.get()), 
                        Units.degreesToRadians(yaw.get()))));

        Logger.recordOutput("RobotVisualizer/coral", Pose3d.kZero.plus(coral));
    }
}
