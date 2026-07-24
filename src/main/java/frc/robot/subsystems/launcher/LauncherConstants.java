package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {
    public enum LauncherState {
        OFF,
        MANUAL,
        SELF_DIRECTING;
    }

    public static final TalonFXConfiguration getLauncherConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
    }

    public static final int LAUNCHER_ONE_ID = 21;
    public static final int LAUNCHER_TWO_ID = 22;

    public static final double LAUNCHER_GEAR_RATIO = 18. / 36; // 0.5:1
    public static final double LAUNCHER_P_COEFFICIENT = 2 * Math.PI / LAUNCHER_GEAR_RATIO;

    public static final double LAUNCHER_MASS = Units.lbsToKilograms(6);
    public static final double LAUNCHER_RADIUS = Units.inchesToMeters(2);
    public static final double LAUNCHER_MOI = 0.0005;

    public static final DCMotor LAUNCHER_MOTORS = DCMotor.getKrakenX60(2);

    // 50% of tangential velocity is transferred to game piece
    public static final double EFFICIENCY = 0.5;
    // ~32 m/s
    public static final double TANGENTIAL_VELOCITY_AT_12V = LAUNCHER_P_COEFFICIENT * LAUNCHER_RADIUS * EFFICIENCY;
}
