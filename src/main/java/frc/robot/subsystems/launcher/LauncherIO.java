package frc.robot.subsystems.launcher;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    @AutoLog
    static class LauncherIOInputs {
        public MotorData launcherOneData = new MotorData();
        public MotorData launcherTwoData = new MotorData();
    }

    default void updateInputs(LauncherIOInputs inputs) {}

    default void runVolts(double volts) {}
}