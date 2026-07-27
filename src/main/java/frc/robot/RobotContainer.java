// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOReal;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.crystalcaverns.Colors.CrystalColor;
import frc.robot.util.Cameras;
import frc.robot.util.crystalcaverns.CrystalCavernsArena;
import frc.robot.util.crystalcaverns.FieldConstants;
import java.util.function.Supplier;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final Arm arm;
    private final EndEffector endEffector;
    private final Launcher launcher;

    public final RobotVisualizer visualizer;

    private final AutoAlign align;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);
    private static final CommandXboxController opController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

//     private Runnable gamePieceVis = () -> {};

    private Supplier<Optional<Pose3d>> heldCrystalPoseSupplier;
    private Supplier<CrystalColor> heldCrystalColorSupplier;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {
                        });
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                launcher = new Launcher(new LauncherIOReal());
                align = new AutoAlign(drive::getPose);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                SimulatedArena.overrideInstance(new CrystalCavernsArena());

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft,
                                driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight,
                                driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft,
                                driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight,
                                driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive, new VisionIOFake(driveSimulation::getSimulatedDriveTrainPose));
                arm = new Arm(new ArmIOSim());
                launcher = new Launcher(new LauncherIOSim());

                EndEffectorIOSim eeSim = new EndEffectorIOSim(
                        driveSimulation, 
                        () -> arm.getArmPosition().getWristTransform(), 
                        () -> arm.getWristAngleRads() > Units.degreesToRadians(125), 
                        launcher::getAngularVelocityRadPerSec);

                // gamePieceVis = eeSim.getGamePieceVizRunnable();
                heldCrystalPoseSupplier = () -> eeSim.getHeldGamePiecePose();
                heldCrystalColorSupplier = () -> eeSim.getHeldCrystalColor();
                endEffector = new EndEffector(eeSim);

                align = new AutoAlign(driveSimulation::getSimulatedDriveTrainPose);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        (pose) -> {
                        });
                vision = new Vision(drive, new VisionIO() {
                }, new VisionIO() {
                });
                arm = new Arm(new ArmIO() {
                });
                endEffector = new EndEffector(new EndEffectorIO() {
                });
                launcher = new Launcher(new LauncherIO() {
                });
                align = new AutoAlign(drive::getPose);

                break;
        }

        launcher.setDistanceToPassingTargetSupplier(align::getDistanceToPassingTarget);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        visualizer = new RobotVisualizer(arm::getArmPosition);

        // Configure the button bindings
        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a
     * {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                true));

        endEffector.setDefaultCommand(endEffector.intake());

        // --- Driver Controls ---
        // controller
        //         .povUp()
        //         .whileTrue(new ConditionalCommand(
        //                 align.netAlign(drive), align.stationAlign(drive),
        //                 endEffector::hasAlgae));
        // controller.povDown().whileTrue(align.reefAlignMid(drive));
        // controller.povLeft().whileTrue(align.caveAlignUpper(drive));
        // controller.povRight().whileTrue(align.caveAlignLower(drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to
                                                               // actual robot pose
                                                               // during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero
                                                                                                       // gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        controller
                .leftBumper()
                .whileTrue(arm.followStateSupplier(() -> ArmState.GROUND_INTAKE))
                .onFalse(arm.followStateSupplier(
                            () -> inAllianceZone() && endEffector.hasGamePiece() ? ArmState.HOLD : ArmState.STOWED)
                    .andThen(new ConditionalCommand(
                        launcher.setSelfDirecting(), Commands.none(), () -> endEffector.hasCrystal() && !inAllianceZone())));

        controller
                .rightBumper()
                .whileTrue(new DeferredCommand(
                        () -> switch (arm.getState()) {
                                case HANDOFF -> endEffector.outtakeHandoff();
                                case BOTTOM_CLASSIFIER, TOP_CLASSIFIER -> endEffector.outtakeClassifier();
                                default -> endEffector.outtake(); 
                        },
                        Set.of(endEffector)))
                .onFalse(launcher.setOff());

        controller
                .a()
                .onTrue(arm.applyState(ArmState.TOP_CLASSIFIER))
                .whileTrue(align.classifierAlign(drive));

        controller
                .x()
                .onTrue(arm.applyState(ArmState.HANDOFF).andThen(launcher.setSelfDirecting()))
                .whileTrue(align.passAlign(
                        drive, 
                        () -> -controller.getLeftY(), 
                        () -> -controller.getLeftX()));
        
        controller
                .b()
                .onTrue(arm.followStateSupplier(
                                () -> ArmState.of(1, align.isForwards())))
                .whileTrue(align.caveAlignLower(drive));

        controller
                .y()
                .onTrue(arm.followStateSupplier(
                                () -> ArmState.of(2, align.isForwards())))
                .whileTrue(align.caveAlignUpper(drive));

        controller
                .povLeft()
                .onTrue(arm.applyState(ArmState.MINE))
                .whileTrue(align.mineAlign(drive));

        // --- Operator Controls ---
        opController.povLeft().onTrue(arm.applyState(ArmState.TUNABLE));
        opController.povRight().onTrue(arm.applyState(ArmState.BOTTOM_CLASSIFIER));
        opController.povDown().onTrue(arm.applyState(ArmState.STOWED)); 
        opController.rightBumper().onTrue(launcher.setManual());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        drive.setPose(new Pose2d(14.77, 2.158, Rotation2d.k180deg));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        if (SimulatedArena.getInstance() instanceof CrystalCavernsArena ccArena) {
                ccArena.logGamePieces(heldCrystalPoseSupplier.get(), heldCrystalColorSupplier.get());
        }
        Cameras.log(driveSimulation.getSimulatedDriveTrainPose());
        // gamePieceVis.run();
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public boolean inAllianceZone() {
        return (drive.getPose().getX() > FieldConstants.FIELD_LENGTH / 2.0) == isRedAlliance();
    }
}
