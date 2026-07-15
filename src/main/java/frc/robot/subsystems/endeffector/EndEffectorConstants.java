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
            INTAKE(-4, 0),
            OUTTAKE(12, 0),
            OUTTAKE_CLASSIFIER(12, 6),
            INDEXING_FWD(-2, 0),
            INDEXING_BWD(2, 0),
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
            CLASSIFIER,
            NONCLASSIFIER
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
