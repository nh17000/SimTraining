package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.subsystems.endeffector.EndEffectorConstants.GamePieceMode;
import frc.robot.util.PhoenixUtil;

public class ArmConstants {
    public enum ArmState {
        STOWED(0, 0, 125),
        UPPER_CRYSTAL(40.5, 38.5, -40.5),
        LOWER_CRYSTAL(45.5, 13.5, -45.5),
        UPPER_CRYSTAL_BACKWARDS(120, 41.5, 60),
        LOWER_CRYSTAL_BACKWARDS(120, 5, 60),
        JEWELRY(20, 2.5, -15),
        JEWELRY_PREP(15, 2.5, -15),
        GROUND_INTAKE(0, 0, -2),
        GEMSTONE_INTAKE(29.5, 42.5, -29.5),
        TOP_CLASSIFIER(64, 14, -45),
        BOTTOM_CLASSIFIER(50, 2.5, -35),
        HIGH_ALGAE_BACKWARDS(100, 16, 80),
        HOLD_CORAL(60, 0, -40),
        HOLD_ALGAE(90, 0, -20),
        TUNABLE(0, 0, 125);

        public final ArmPosition position;

        private ArmState(double pivotDegrees, double extensionInches, double wristDegrees) {
            this.position = new ArmPosition(
                    Units.degreesToRadians(pivotDegrees),
                    Units.inchesToMeters(extensionInches),
                    Units.degreesToRadians(wristDegrees));
        }

        public static ArmState of(int level, boolean isForwards) {
            // return switch (mode) {
            //     case VERTICAL_CORAL -> switch (level) {
            //         case 4 -> isForwards ? ArmState.L4 : ArmState.L4_BACKWARDS;
            //         case 3 -> isForwards ? ArmState.L3 : ArmState.L3_BACKWARDS;
            //         case 2 -> isForwards ? ArmState.L2 : ArmState.L2_BACKWARDS;
            //         default -> ArmState.STOWED;
            //     };
            //     case HORIZONTAL_CORAL -> switch (level) {
            //         case 4 -> ArmState.L1_TOP;
            //         case 3 -> ArmState.L1_FRONT;
            //         case 2 -> ArmState.L1_BASE;
            //         default -> ArmState.STOWED;
            //     };
            //     case ALGAE -> switch (level) {
            //         case 4 -> ArmState.NET_PREP;
            //         case 3 -> isForwards ? ArmState.HIGH_ALGAE : ArmState.HIGH_ALGAE_BACKWARDS;
            //         case 2 -> isForwards ? ArmState.LOW_ALGAE : ArmState.LOW_ALGAE_BACKWARDS;
            //         case 1 -> ArmState.PROCESSOR;
            //         default -> ArmState.STOWED;
            //     };
            // };
            return switch (level) {
                case 4 -> isForwards ? ArmState.UPPER_CRYSTAL : ArmState.UPPER_CRYSTAL_BACKWARDS;
                case 3 -> isForwards ? ArmState.LOWER_CRYSTAL : ArmState.LOWER_CRYSTAL_BACKWARDS;
                case 2 -> isForwards ? ArmState.GEMSTONE_INTAKE : ArmState.HIGH_ALGAE_BACKWARDS;
                case 1 -> isForwards ? ArmState.BOTTOM_CLASSIFIER : ArmState.TOP_CLASSIFIER;
                default -> ArmState.STOWED;
            };
        }

        public static ArmState groundIntake(GamePieceMode mode) {
            return ArmState.GROUND_INTAKE;
            // return switch (mode) {
            //     case VERTICAL_CORAL -> ArmState.VERTICAL_CORAL_GROUND_INTAKE;
            //     case HORIZONTAL_CORAL -> ArmState.HORIZONTAL_CORAL_GROUND_INTAKE;
            //     case ALGAE -> ArmState.ALGAE_GROUND_INTAKE;
            // };
        }

        public static ArmState hold(GamePieceMode mode) {
            return ArmState.HOLD_CORAL;
            // return switch (mode) {
            //     case VERTICAL_CORAL -> ArmState.HOLD_CORAL;
            //     case HORIZONTAL_CORAL -> ArmState.L1_BASE;
            //     case ALGAE -> ArmState.HOLD_ALGAE;
            // };
        }
    }

    public static final TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 25;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 25;

        config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRadians(1000) / PIVOT_P_COEFFICIENT;
        config.MotionMagic.MotionMagicAcceleration = Units.degreesToRadians(600) / PIVOT_P_COEFFICIENT;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 0.6;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        return config;
    }

    public static final TalonFXConfiguration getExtensionConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30;

        config.MotionMagic.MotionMagicCruiseVelocity = Units.inchesToMeters(200) / EXTENSION_P_COEFFICIENT;
        config.MotionMagic.MotionMagicAcceleration = Units.inchesToMeters(400) / EXTENSION_P_COEFFICIENT;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        return config;
    }

    public static final TalonFXConfiguration getWristConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30;

        config.MotionMagic.MotionMagicCruiseVelocity = 20;
        config.MotionMagic.MotionMagicAcceleration = 75;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.088;
        config.Slot0.kA = 0.001;
        config.Slot0.kP = 0.05;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        return config;
    }

    public static final int PIVOT_ONE_ID = 20;
    public static final int PIVOT_TWO_ID = 21;
    public static final int PIVOT_THREE_ID = 22;

    public static final int EXTENSION_ONE_ID = 30;
    public static final int EXTENSION_TWO_ID = 31;
    public static final int EXTENSION_THREE_ID = 32;

    public static final int WRIST_ID = 40;

    public static final double PIVOT_GEAR_RATIO = (60. / 12.) * (60. / 16.) * (58. / 9.); // 120.83
    public static final double EXTENSION_GEAR_RATIO = (32. / 16.) * (40. / 26.) * (50. / 20.) * (62. / 76.); // 6.28
    public static final double WRIST_GEAR_RATIO = (50. / 9.) * (38. / 12.) * (38. / 12.); // 55.71

    public static final double ARM_MASS_KG = Units.lbsToKilograms(20);
    public static final double ARM_SHOULDER_TO_WRIST_LENGTH = VisualizerConstants.WRIST_OFFSET.getNorm();

    public static final double PIVOT_P_COEFFICIENT = 2 * Math.PI / PIVOT_GEAR_RATIO;
    public static final double PIVOT_MIN_ANGLE = 0;
    public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(120);
    public static final DCMotor PIVOT_MOTORS = DCMotor.getKrakenX60(3);

    public static final double EXTENSION_DRUM_RADIUS = Units.inchesToMeters(0.25 * 16.0 / Math.PI * 0.5); // ~0.64"
    public static final double EXTENSION_P_COEFFICIENT = 2.0 * Math.PI * EXTENSION_DRUM_RADIUS / EXTENSION_GEAR_RATIO;
    public static final double EXTENSION_MIN_LENGTH = 0;
    public static final double EXTENSION_MAX_LENGTH = Units.inchesToMeters(42);
    public static final DCMotor EXTENSION_MOTORS = DCMotor.getKrakenX60(3);

    public static final double WRIST_P_COEFFICIENT = 2 * Math.PI / WRIST_GEAR_RATIO;
    public static final double WRIST_MASS_KG = Units.lbsToKilograms(2);
    public static final double WRIST_LENGTH = Units.inchesToMeters(6);
    public static final double WRIST_STARTING_ANGLE = Units.degreesToRadians(125);
    public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-90);
    public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(150);
    public static final DCMotor WRIST_MOTOR = PhoenixUtil.getOldKrakenX44(1);
}
