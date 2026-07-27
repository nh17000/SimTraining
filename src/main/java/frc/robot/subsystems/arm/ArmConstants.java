package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisualizerConstants;

public class ArmConstants {
    public enum ArmState {
        STOWED(0, 0, 0),
        UPPER_CRYSTAL(36.5, 25, -36.5),
        LOWER_CRYSTAL(36.5, 0, -36.5),
        UPPER_CRYSTAL_BACKWARDS(148, 24, 32),
        LOWER_CRYSTAL_BACKWARDS(148, 0, 35),
        GROUND_INTAKE(0, 0, -70),
        TOP_CLASSIFIER(55, 6, -60),
        BOTTOM_CLASSIFIER(36.5, 0, -47.5),
        HOLD(60, 0, -36.5),
        MINE(40, 0, -12.5),
        HANDOFF(0, 0, 128.5),
        TUNABLE(0, 0, 125);

        public final ArmPosition position;

        private ArmState(double pivotDegrees, double extensionInches, double wristDegrees) {
            this.position = new ArmPosition(
                    Units.degreesToRadians(pivotDegrees),
                    Units.inchesToMeters(extensionInches),
                    Units.degreesToRadians(wristDegrees));
        }

        public static ArmState of(int level, boolean isForwards) {
            return switch (level) {
                case 2 -> isForwards ? ArmState.UPPER_CRYSTAL : ArmState.UPPER_CRYSTAL_BACKWARDS;
                case 1 -> isForwards ? ArmState.LOWER_CRYSTAL : ArmState.LOWER_CRYSTAL_BACKWARDS;
                default -> ArmState.STOWED;
            };
        }
    }

    public static final TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 45;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 45;

        config.MotionMagic.MotionMagicCruiseVelocity = 75;
        config.MotionMagic.MotionMagicAcceleration = 275;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 2.5;
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

        config.MotionMagic.MotionMagicCruiseVelocity = 80;
        config.MotionMagic.MotionMagicAcceleration = 325;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 1.5;
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

        config.MotionMagic.MotionMagicCruiseVelocity = 50;
        config.MotionMagic.MotionMagicAcceleration = 200;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 1.0;
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

    public static final double PIVOT_GEAR_RATIO = 130; // was 120.83
    public static final double EXTENSION_GEAR_RATIO = 4; // was 6.28
    public static final double WRIST_GEAR_RATIO = 54. / 8.; // 6.75

    public static final double EXTENSION_TUBE_MASS = Units.lbsToKilograms(1.5);
    public static final double WRIST_MASS_KG = Units.lbsToKilograms(5);
    public static final double ARM_STAGE0_MASS = Units.lbsToKilograms(20.5); // most of this is the launcher

    public static final double EXTENSION_MASS_KG = EXTENSION_TUBE_MASS + WRIST_MASS_KG;
    public static final double TOTAL_MASS_KG = ARM_STAGE0_MASS + EXTENSION_TUBE_MASS + WRIST_MASS_KG;
    
    public static final double ARM_SHOULDER_TO_WRIST_LENGTH = VisualizerConstants.WRIST_OFFSET.getNorm();

    public static final double ARM_STAGE0_MOI = (1. / 3.) * ARM_STAGE0_MASS * Math.pow(ARM_SHOULDER_TO_WRIST_LENGTH, 2);
    public static final double EXTENSION_TUBE_I_CM = (1. / 12.) * EXTENSION_TUBE_MASS * Math.pow(ARM_SHOULDER_TO_WRIST_LENGTH / 2.0, 2);

    public static final double PIVOT_P_COEFFICIENT = 2 * Math.PI / PIVOT_GEAR_RATIO;
    public static final double PIVOT_MIN_ANGLE = 0;
    public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(160);
    public static final DCMotor PIVOT_MOTORS = DCMotor.getKrakenX60Foc(3);

    public static final double EXTENSION_DRUM_RADIUS = Units.inchesToMeters(0.8); //Units.inchesToMeters(0.25 * 16.0 / Math.PI * 0.5); // ~0.64"
    public static final double EXTENSION_P_COEFFICIENT = 2.0 * Math.PI * EXTENSION_DRUM_RADIUS / EXTENSION_GEAR_RATIO;
    public static final double EXTENSION_MIN_LENGTH = 0;
    public static final double EXTENSION_MAX_LENGTH = Units.inchesToMeters(30.5);
    public static final DCMotor EXTENSION_MOTORS = DCMotor.getKrakenX60Foc(2);

    public static final double WRIST_P_COEFFICIENT = 2 * Math.PI / WRIST_GEAR_RATIO;
    public static final double WRIST_LENGTH = Units.inchesToMeters(6);
    public static final double WRIST_STARTING_ANGLE = Units.degreesToRadians(-57.737877);
    public static final double WRIST_MIN_ANGLE = Units.degreesToRadians(-165);
    public static final double WRIST_MAX_ANGLE = Units.degreesToRadians(130); // 105
    public static final DCMotor WRIST_MOTOR = DCMotor.getKrakenX60Foc(1);
}
