package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class EndEffectorConstants {
    public enum EEState {
            VERTICAL_CORAL_INTAKE(-4, 0),
            VERTICAL_CORAL_OUTTAKE_FWD(-12, 0),
            VERTICAL_CORAL_OUTTAKE_BWD(12, 0),
            HORIZONTAL_CORAL_INTAKE(-4, 4),
            HORIZONTAL_CORAL_OUTTAKE(2, 2),
            ALGAE_INTAKE(0, 8),
            ALGAE_OUTTAKE(0, -12),
            INDEXING_FWD(-2, 0),
            INDEXING_BWD(2, 0),
            INDEXING_HORIZONTAL(0, -1), // todo: left & right
            OFF(0, 0);

            public final double leftVolts;
            public final double rightVolts;
            public final double topVolts;

            private EEState(double leftVolts, double topVolts) {
                this.leftVolts = leftVolts;
                this.rightVolts = -leftVolts;
                this.topVolts = topVolts;
            }
        }

        public enum GamePieceMode {
            HORIZONTAL_CORAL,
            VERTICAL_CORAL,
            ALGAE
        }

        public static final TalonFXConfiguration getLRConfigs() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            // disable roller current limits in sim
            config.CurrentLimits.SupplyCurrentLimitEnable = Constants.currentMode != Mode.SIM;
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.StatorCurrentLimitEnable = Constants.currentMode != Mode.SIM;
            config.CurrentLimits.StatorCurrentLimit = 30;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final TalonFXConfiguration getTopConfigs() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 100;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 100;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final CANrangeConfiguration getCANRangeConfigs() {
            CANrangeConfiguration config = new CANrangeConfiguration();

            config.ProximityParams.ProximityHysteresis = 0.01;
            config.ProximityParams.ProximityThreshold = 0.07;
            config.ProximityParams.MinSignalStrengthForValidMeasurement = 8000;

            return config;
        }

        public static final int LEFT_ID = 41;
        public static final int RIGHT_ID = 42;
        public static final int TOP_ID = 43;

        public static final int FRONT_CAN_RANGE_ID = 44;
        public static final int BACK_CAN_RANGE_ID = 45;
        public static final int FRONT_LEFT_CAN_RANGE_ID = 46;
        public static final int FRONT_RIGHT_CAN_RANGE_ID = 47;

        public static final double TRANSLATIONAL_TOLERANCE = Units.inchesToMeters(16);

        public static final double INTAKING_TIME = 0.5;
        public static final double DROP_COOLDOWN = 2.0;

        public static final double LR_GEAR_RATIO = (40. / 12.) * (40. / 14.);
        public static final double LR_MASS = Units.lbsToKilograms(0.5);
        public static final double LR_RADIUS = Units.inchesToMeters(2);
        public static final double LR_MOI = 1.0 / 2.0 * LR_MASS * LR_RADIUS * LR_RADIUS;

        public static final DCMotor LR_MOTOR = DCMotor.getKrakenX44(1);
}
