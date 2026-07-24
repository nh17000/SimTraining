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
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.util.crystalcaverns.MineSim;
import frc.robot.util.crystalcaverns.Colors.CrystalColor;
import frc.robot.util.crystalcaverns.Colors;
import frc.robot.util.crystalcaverns.CrystalOnField;
import frc.robot.util.crystalcaverns.CrystalOnFly;
import frc.robot.util.crystalcaverns.FieldConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

public class EndEffectorIOSim extends EndEffectorIOTalonFX {
    private boolean hasCrystal = true;

    @Getter
    private CrystalColor heldCrystalColor = Colors.getRandomCrystalColor();

    private static final Pose3d HIDDEN_CRYSTAL_POS = 
            new Pose3d(FieldConstants.CENTER.getX(), FieldConstants.CENTER.getY(), -0.5, Rotation3d.kZero);

    private Timer intakingTimer = new Timer();

    private Pose3d intookGamePiecePrevPose = HIDDEN_CRYSTAL_POS;

    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    EndEffectorConstants.LR_MOTOR, EndEffectorConstants.LR_MOI, EndEffectorConstants.LR_GEAR_RATIO),
            EndEffectorConstants.LR_MOTOR);

    private static final double MIN_POS = 0.195;
    private static final double MAX_EE_POS = 0.315;
    private static final double MAX_LAUNCHER_POS = 0.55;
    private static final double INTAKE_POS = 0.3;
    
    private static final double POS_TOLERANCE = 0.01;
    
    @AutoLogOutput(key = "EndEffector/Roller Theta")
    private double rollerAngularPosition = 0.0;
    
    @AutoLogOutput(key = "EndEffector/Crystal Pos")
    private double crystalPos = INTAKE_POS;

    private final IntakeSimulation crystalGroundIntakeSim;

    private final TalonFXSimState leftSimState;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<Transform3d> wristTransformSupplier;
    private BooleanSupplier handoffSupplier;
    private DoubleSupplier launcherRPSSupplier;

    // private final LoggedTunableNumber x = new LoggedTunableNumber("Test/x", 0.0);
    // private final LoggedTunableNumber y = new LoggedTunableNumber("Test/y", 0.0);
    // private final LoggedTunableNumber z = new LoggedTunableNumber("Test/z", 0.0);
    // private final LoggedTunableNumber roll = new LoggedTunableNumber("Test/r", 0.0);
    // private final LoggedTunableNumber pitch = new LoggedTunableNumber("Test/p", 0.0);
    // private final LoggedTunableNumber yaw = new LoggedTunableNumber("Test/yaw", 0.0);
    // private final LoggedTunableNumber r2 = new LoggedTunableNumber("Test/r2", 0.0);
    // private final LoggedTunableNumber p2 = new LoggedTunableNumber("Test/p2", 0.0);
    // private final LoggedTunableNumber yaw2 = new LoggedTunableNumber("Test/yaw2", 0.0);

    public EndEffectorIOSim(
            AbstractDriveTrainSimulation driveSimulation, Supplier<Transform3d> wristTransformSupplier, BooleanSupplier canHandoffSupplier, DoubleSupplier launcherRPSSupplier) {

        crystalGroundIntakeSim = IntakeSimulation.OverTheBumperIntake(
                "Crystal", driveSimulation, Inches.of(12), Inches.of(12), IntakeSimulation.IntakeSide.FRONT, 1);

        crystalGroundIntakeSim.setCustomIntakeCondition((GamePieceOnFieldSimulation gp) -> {
            if (gp instanceof CrystalOnField crystal) {
                heldCrystalColor = crystal.color;
            }
            return true;
        });

        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;
        this.wristTransformSupplier = wristTransformSupplier;
        this.handoffSupplier = canHandoffSupplier;
        this.launcherRPSSupplier = launcherRPSSupplier;

        leftSimState = left.getSimState();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        boolean handoff = handoffSupplier.getAsBoolean();

        updateRollerSim(Constants.LOOP_PERIOD, handoff);
        updateGamePieces(inputs.leftData.appliedVolts(), inputs.topData.appliedVolts(), handoff);
        updateCANRangeSims();

        inputs.hasCrystal = hasCrystal;
        inputs.hasGemstone = false;
    }

    // public Runnable getGamePieceVizRunnable() {
    //     return () -> visualizeGamePieces();
    // }

    public Optional<Pose3d> getHeldGamePiecePose() {
        if (!hasCrystal) return Optional.empty();
        return Optional.of(interpolateHeldPose(getHeldCrystalPose()));
    }

    private void updateRollerSim(double dtSeconds, boolean handoff) {
        leftSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        rollerSim.setInputVoltage(leftSimState.getMotorVoltage());
        rollerSim.update(dtSeconds);

        double deltaTheta = dtSeconds * rollerSim.getAngularVelocityRadPerSec();
        rollerAngularPosition += deltaTheta;
        if (hasCrystal) {
            crystalPos = MathUtil.clamp(
                    crystalPos + EndEffectorConstants.LR_RADIUS * deltaTheta, // * MULT,
                    MIN_POS - POS_TOLERANCE,
                    (handoff ? MAX_LAUNCHER_POS : MAX_EE_POS) + POS_TOLERANCE);
        } else {
            crystalPos = INTAKE_POS;
        }

        leftSimState.setRawRotorPosition(
                Units.radiansToRotations(rollerAngularPosition) * EndEffectorConstants.LR_GEAR_RATIO);
        leftSimState.setRotorVelocity(
                Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()) * EndEffectorConstants.LR_GEAR_RATIO);
    }

    private void updateGamePieces(double leftVolts, double topVolts, boolean handoff) {
        if (leftVolts < -0.5) { // intake
            dropFromCrystalStation(); // drop crystal from the station

            if (!hasCrystal && getHeldCrystalTransform().getMeasureZ().lt(Inches.of(6))) {
                // intake crystal on the ground
                crystalGroundIntakeSim.startIntake();
            } else { // intake crystal in the air
                crystalGroundIntakeSim.stopIntake();
                intakeCrystalProjectiles();
            }
        } else if (leftVolts > 0.5) {
            crystalGroundIntakeSim.stopIntake();
        }

        if (hasCrystal != crystalGroundIntakeSim.getGamePiecesAmount() > 0) {
            if (!hasCrystal) {
                hasCrystal = true; // game piece intook
            } else crystalGroundIntakeSim.addGamePieceToIntake(); // preload
        }

        if (crystalPos > (handoff ? MAX_LAUNCHER_POS : MAX_EE_POS) - POS_TOLERANCE) {
            shootCrystal(handoff, topVolts > 0.5);
            crystalPos = INTAKE_POS;
        }
    }

    private void updateCANRangeSims() {
        boolean isFrontTripped = hasCrystal;// && MathUtil.isNear(FRONT_CAN_RANGE_POS, crystalPos, CORAL_LENGTH / 2.0);
        boolean isBackTripped = hasCrystal;// && MathUtil.isNear(BACK_CAN_RANGE_POS, crystalPos, CORAL_LENGTH / 2.0);
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
                    SimulatedArena.getInstance().removeProjectile(gamePiece);
                    intookGamePiecePrevPose = gamePiecePose;
                    hasCrystal = true;
                    heldCrystalColor = ((CrystalOnFly) gamePiece).color;
                    crystalPos = INTAKE_POS;
                    intakingTimer.restart();
                    break;
                }
            }
        }
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

    // private void visualizeGamePieces() {
    //     for (CrystalColor color : CrystalColor.values()) {
    //         Logger.recordOutput(
    //             "FieldSimulation/Held " + color.toString() + " Crystal", 
    //             (hasCrystal && heldCrystalColor == color) 
    //                 ? interpolateHeldPose(getHeldCrystalPose()) 
    //                 : HIDDEN_CRYSTAL_POS);
    //     }
    // }

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
                // .plus(new Transform3d(
                //     new Translation3d(x.get(), y.get(), z.get())
                //             .rotateBy(new Rotation3d(
                //                 Units.degreesToRadians(r2.get()), 
                //                 Units.degreesToRadians(p2.get()), 
                //                 Units.degreesToRadians(yaw2.get()))),
                //     new Rotation3d(
                //         Units.degreesToRadians(roll.get()), 
                //         Units.degreesToRadians(pitch.get()), 
                //         Units.degreesToRadians(yaw.get()))));
                // .plus(VisualizerConstants.CRYSTAL_TRANSFORM)
                .plus(new Transform3d(
                    new Translation3d(crystalPos, 0, 0.015)
                        .rotateBy(new Rotation3d(0, Units.degreesToRadians(45), 0)), 
                    new Rotation3d(0, Units.degreesToRadians(45), 0)));
    }

    private Pose3d getHeldCrystalPose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        Transform3d crystalTransform = getHeldCrystalTransform();
        return robotPose.transformBy(crystalTransform);
    }

    private void shootCrystal(boolean handoff, boolean classifier) {
        if (!hasCrystal) return;

        Transform3d eeTransform = getHeldCrystalTransform();

        double ejectVel;
        if (handoff) {
            ejectVel = launcherRPSSupplier.getAsDouble() * LauncherConstants.LAUNCHER_RADIUS * LauncherConstants.EFFICIENCY;
        } else {
            ejectVel = rollerSim.getAngularVelocityRadPerSec() * EndEffectorConstants.LR_RADIUS;
            if (!classifier) ejectVel *= 0.4;
        }

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new CrystalOnFly(
                        poseSupplier.get().getTranslation(),
                        eeTransform.getTranslation().toTranslation2d(),
                        chassisSpeedsSupplier.get(),
                        poseSupplier.get().getRotation(),
                        eeTransform.getMeasureZ(),
                        MetersPerSecond.of(ejectVel),
                        eeTransform.getRotation().getMeasureAngle(),
                        heldCrystalColor));

        hasCrystal = false;
        crystalGroundIntakeSim.obtainGamePieceFromIntake();
    }


    private void dropFromCrystalStation() {
        if (hasCrystal) return;

        if (SimulatedArena.getInstance().gamePiecesOnField().size() > 30) return;

        Pose2d eePose = getHeldCrystalPose().toPose2d();
        Pose2d nearestCS = MineSim.getMine();

        if (eePose.minus(nearestCS).getTranslation().getNorm() < EndEffectorConstants.TRANSLATIONAL_TOLERANCE) {
            MineSim.drop(false);
        } else {
            MineSim.drop(true);
        }
    }
}
