package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.util.crystalcaverns.GemstoneHandler;
import frc.robot.util.crystalcaverns.MineSim;
import frc.robot.util.crystalcaverns.CrystalOnFly;
import frc.robot.util.crystalcaverns.GemstoneOnFly;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim extends EndEffectorIOTalonFX {
    private boolean hasCrystal = true;
    private boolean hasGemstone = false;

    private Timer intakingTimer = new Timer();

    private Pose3d intookGamePiecePrevPose = Pose3d.kZero;

    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    EndEffectorConstants.LR_MOTOR, EndEffectorConstants.LR_MOI, EndEffectorConstants.LR_GEAR_RATIO),
            EndEffectorConstants.LR_MOTOR);

    private static final double MULT = 0.01;
    private static final double OFFSET = 0.225;

    private static final double MIN_POS = -0.1 * MULT + OFFSET; 
    private static final double MAX_POS = 0.35 * MULT + OFFSET;
    private static final double INTAKE_POS = 0.25 * MULT + OFFSET;

    private static final double POS_TOLERANCE = 0.01 * MULT;

    private static final double CORAL_LENGTH = Units.inchesToMeters(11.875) * MULT;
    private static final double FRONT_CAN_RANGE_POS = 0.175 * MULT + OFFSET;
    private static final double BACK_CAN_RANGE_POS = 0.05 * MULT + OFFSET;

    @AutoLogOutput(key = "EndEffector/Roller Theta")
    private double rollerAngularPosition = 0.0;

    @AutoLogOutput(key = "EndEffector/Crystal Pos")
    private double crystalPos = INTAKE_POS;

    private final IntakeSimulation crystalGroundIntakeSim;
    private final IntakeSimulation gemstoneGroundIntakeSim;

    private final TalonFXSimState leftSimState;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<Transform3d> wristTransformSupplier;

    // private final LoggedTunableNumber x = new LoggedTunableNumber("Gemstone/x", 0.0);
    // private final LoggedTunableNumber y = new LoggedTunableNumber("Gemstone/y", 0.0);
    // private final LoggedTunableNumber z = new LoggedTunableNumber("Gemstone/z", 0.0);
    // private final LoggedTunableNumber roll = new LoggedTunableNumber("Gemstone/r", 0.0);
    // private final LoggedTunableNumber pitch = new LoggedTunableNumber("Gemstone/p", 0.0);
    // private final LoggedTunableNumber yaw = new LoggedTunableNumber("Gemstone/yaw", 0.0);
    // private final LoggedTunableNumber r2 = new LoggedTunableNumber("Gemstone/r2", 0.0);
    // private final LoggedTunableNumber p2 = new LoggedTunableNumber("Gemstone/p2", 0.0);
    // private final LoggedTunableNumber yaw2 = new LoggedTunableNumber("Gemstone/yaw2", 0.0);

    public EndEffectorIOSim(
            AbstractDriveTrainSimulation driveSimulation, Supplier<Transform3d> wristTransformSupplier) {

        crystalGroundIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Crystal", driveSimulation, Inches.of(12), Inches.of(6), IntakeSimulation.IntakeSide.FRONT, 1);
        gemstoneGroundIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Gemstone", driveSimulation, Inches.of(12), Inches.of(6), IntakeSimulation.IntakeSide.FRONT, 1);

        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;
        this.wristTransformSupplier = wristTransformSupplier;

        leftSimState = left.getSimState();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        updateRollerSim(Constants.LOOP_PERIOD);
        updateGamePieces(inputs.leftData.appliedVolts(), inputs.topData.appliedVolts());
        updateCANRangeSims();

        inputs.hasCrystal = hasCrystal;
        inputs.hasGemstone = hasGemstone;
    }

    private void updateRollerSim(double dtSeconds) {
        leftSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        rollerSim.setInputVoltage(leftSimState.getMotorVoltage());
        rollerSim.update(dtSeconds);

        double deltaTheta = dtSeconds * rollerSim.getAngularVelocityRadPerSec();
        rollerAngularPosition += deltaTheta;
        if (hasCrystal) {
            crystalPos = MathUtil.clamp(
                    crystalPos + EndEffectorConstants.LR_RADIUS * deltaTheta * MULT,
                    MIN_POS - POS_TOLERANCE,
                    MAX_POS + POS_TOLERANCE);
        } else {
            crystalPos = 0.0;
        }

        leftSimState.setRawRotorPosition(
                Units.radiansToRotations(rollerAngularPosition) * EndEffectorConstants.LR_GEAR_RATIO);
        leftSimState.setRotorVelocity(
                Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()) * EndEffectorConstants.LR_GEAR_RATIO);
    }

    private void updateGamePieces(double leftVolts, double topVolts) {
        // if (Math.abs(leftVolts) < 0.1) { // gemstone
        //     crystalGroundIntakeSim.stopIntake();

        //     if (topVolts > 0.5) { // gemstone intake
        //         if (getHeldGemstoneTransform().getMeasureZ().lt(Inches.of(16))) {
        //             // intake gemstone on the ground
        //             gemstoneGroundIntakeSim.startIntake();
        //         } else { // intake gemstone on the reef
        //             gemstoneGroundIntakeSim.stopIntake();
        //             intakeReefGemstone();
        //         }
        //     } else {
        //         gemstoneGroundIntakeSim.stopIntake();
        //         if (topVolts < -0.5) { // gemstone outtake
        //             shootGemstone();
        //         }
        //     }
        // } else { // crystal
        //     horizontal = Math.abs(topVolts) > 0.5; // intake horizontally if top rollers running
        //     gemstoneGroundIntakeSim.stopIntake();

        //     if (leftVolts < -0.5) { // crystal intake
        //         dropFromCrystalStation(); // drop crystal from the station

        //         if (getHeldCrystalTransform().getMeasureZ().lt(Inches.of(8))) {
        //             // intake crystal on the ground
        //             crystalGroundIntakeSim.startIntake();
        //         } else { // intake crystal in the air
        //             crystalGroundIntakeSim.stopIntake();
        //             // intakeCrystalProjectiles();
        //         }
        //     } else {
        //         crystalGroundIntakeSim.stopIntake();
        //     }
        // }

        if (leftVolts < -0.5) { // intake
            dropFromCrystalStation(); // drop crystal from the station

            if (!hasCrystal && !hasGemstone && getHeldCrystalTransform().getMeasureZ().lt(Inches.of(8))) {
                // intake crystal on the ground
                crystalGroundIntakeSim.startIntake();
                gemstoneGroundIntakeSim.startIntake();
            } else { // intake crystal in the air
                crystalGroundIntakeSim.stopIntake();
                // intakeCrystalProjectiles();
                gemstoneGroundIntakeSim.stopIntake();
                intakeReefGemstone();
            }
        } else if (leftVolts > 0.5) {
            crystalGroundIntakeSim.stopIntake();
            gemstoneGroundIntakeSim.stopIntake();
            shootGemstone();
        }

        if (hasCrystal != crystalGroundIntakeSim.getGamePiecesAmount() > 0) {
            if (!hasCrystal) {
                hasCrystal = true; // game piece intook
                crystalPos = INTAKE_POS;
            } else crystalGroundIntakeSim.addGamePieceToIntake(); // preload
        }

        if (hasGemstone != gemstoneGroundIntakeSim.getGamePiecesAmount() > 0) {
            if (!hasGemstone) hasGemstone = true; // game piece intook
            else gemstoneGroundIntakeSim.addGamePieceToIntake(); // preload
        }

        if (crystalPos > MAX_POS - POS_TOLERANCE) {
            shootCrystal(false, topVolts > 0.5);
        } else if (crystalPos < MIN_POS + POS_TOLERANCE) {
            shootCrystal(true, topVolts > 0.5);
        }

        visualizeGamePieces();
    }

    private void updateCANRangeSims() {
        boolean isFrontTripped = hasCrystal && MathUtil.isNear(FRONT_CAN_RANGE_POS, crystalPos, CORAL_LENGTH / 2.0);
        boolean isBackTripped = hasCrystal && MathUtil.isNear(BACK_CAN_RANGE_POS, crystalPos, CORAL_LENGTH / 2.0);
        boolean areSidesTripped = false; // hasCrystal && horizontal;

        frontCANRange.getSimState().setDistance(isFrontTripped ? 0.01 : 1);
        backCANRange.getSimState().setDistance(isBackTripped ? 0.01 : 1);
        frontLeftCANRange.getSimState().setDistance(areSidesTripped ? 0.01 : 1);
        frontRightCANRange.getSimState().setDistance(areSidesTripped ? 0.01 : 1);
    }

    private void intakeCrystalProjectiles() {
        if (hasCrystal) return; // max 1 crystal

        Pose3d intakePose = getHeldCrystalPose(); // the end effector is the intake

        var iterator = SimulatedArena.getInstance().gamePieceLaunched().iterator();
        while (iterator.hasNext()) {
            var gamePiece = iterator.next();
            if (gamePiece instanceof CrystalOnFly) {
                Pose3d gamePiecePose = gamePiece.getPose3d();
                Transform3d diff = intakePose.minus(gamePiecePose);
                Translation3d velocity = gamePiece.getVelocity3dMPS();

                // Only pick up if approaching and within tolerance
                if (checkTolerance(diff) && isApproaching(intakePose, gamePiecePose, velocity)) {
                    iterator.remove();
                    intookGamePiecePrevPose = gamePiecePose;
                    hasCrystal = true;
                    crystalPos = INTAKE_POS;
                    intakingTimer.restart();
                    break;
                }
            }
        }
    }

    private void intakeReefGemstone() {
        if (hasGemstone) return; // max 1 gemstone

        GemstoneHandler.getInstance().intake(getHeldGemstonePose()).ifPresent(gemstone -> {
            intookGamePiecePrevPose =
                    new Pose3d(gemstone.getTranslation(), getHeldGemstonePose().getRotation());
            hasGemstone = true;
            intakingTimer.restart();
        });
    }

    private static boolean checkTolerance(Transform3d difference) {
        return difference.getTranslation().getNorm() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE;
    }

    private static boolean isApproaching(Pose3d intakePose, Pose3d gamePiecePose, Translation3d velocity) {
        Translation3d currentVec = intakePose.getTranslation().minus(gamePiecePose.getTranslation());
        double currentDist = currentVec.getNorm();

        Translation3d futurePosition = gamePiecePose.getTranslation().plus(velocity.times(0.001));
        double futureDist = intakePose.getTranslation().minus(futurePosition).getNorm();

        return futureDist < currentDist;
    }

    private void visualizeGamePieces() {
        Logger.recordOutput(
                "FieldSimulation/Held Crystal", hasCrystal ? interpolateHeldPose(getHeldCrystalPose()) : Pose3d.kZero);
        Logger.recordOutput(
                "FieldSimulation/Held Gemstone", hasGemstone ? interpolateHeldPose(getHeldGemstonePose()) : Pose3d.kZero);
    }

    private Pose3d interpolateHeldPose(Pose3d targetPose) {
        if (intakingTimer.isRunning()) {
            if (intakingTimer.get() > EndEffectorConstants.INTAKING_TIME) {
                intakingTimer.stop();
                intakingTimer.reset();
            } else {
                return intookGamePiecePrevPose.interpolate(
                        targetPose, intakingTimer.get() / EndEffectorConstants.INTAKING_TIME);
            }
        }
        return targetPose;
    }

    private Transform3d getHeldCrystalTransform() {
        return wristTransformSupplier
                .get()
                .plus(VisualizerConstants.CRYSTAL_TRANSFORM)
                .plus(new Transform3d(crystalPos, 0, 0, Rotation3d.kZero));
    }

    private Transform3d getHeldGemstoneTransform() {
        return wristTransformSupplier.get().plus(VisualizerConstants.GEMSTONE_TRANSFORM);
    }

    private Pose3d getHeldCrystalPose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        Transform3d crystalTransform = getHeldCrystalTransform();
        return robotPose.transformBy(crystalTransform);
    }

    private Pose3d getHeldGemstonePose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        return robotPose.transformBy(getHeldGemstoneTransform());
    }

    private void shootCrystal(boolean forwards, boolean classifier) {
        if (!hasCrystal) return;

        Transform3d eeTransform = getHeldCrystalTransform();

        double ejectVel = rollerSim.getAngularVelocityRadPerSec() * EndEffectorConstants.LR_RADIUS;
        // if (horizontal) ejectVel = -Math.abs(ejectVel);
        if (!classifier) ejectVel *= 0; // how to rotate velocity vector

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new CrystalOnFly(
                        poseSupplier.get().getTranslation(),
                        eeTransform.getTranslation().toTranslation2d(),
                        chassisSpeedsSupplier.get(),
                        poseSupplier.get().getRotation(),
                        eeTransform.getMeasureZ(),
                        MetersPerSecond.of(ejectVel),
                        eeTransform.getRotation().getMeasureAngle()));

        hasCrystal = false;
        crystalGroundIntakeSim.obtainGamePieceFromIntake();
    }

    private void shootGemstone() {
        if (!hasGemstone) return;

        Transform3d eeTransform = getHeldGemstoneTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new GemstoneOnFly(
                        poseSupplier.get().getTranslation(),
                        eeTransform.getTranslation().toTranslation2d(),
                        chassisSpeedsSupplier.get(),
                        poseSupplier.get().getRotation(),
                        eeTransform.getMeasureZ(),
                        MetersPerSecond.of(2.910 * 0),
                        eeTransform.getRotation().getMeasureAngle()));

        hasGemstone = false;
        gemstoneGroundIntakeSim.obtainGamePieceFromIntake();
    }

    private void dropFromCrystalStation() {
        if (hasCrystal) return;

        if (SimulatedArena.getInstance().gamePiecesOnField().size() > 30) return;

        Pose2d eePose = getHeldCrystalPose().toPose2d();
        Pose2d nearestCS = MineSim.getNearestStation(eePose);

        if (eePose.minus(nearestCS).getTranslation().getNorm() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE) {
            MineSim.drop(nearestCS, false);
        } else {
            MineSim.drop(nearestCS, true);
        }
    }
}
