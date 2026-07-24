package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.util.PearadoxTalonFX;

public abstract class LauncherIOTalonFX implements LauncherIO {
    protected final PearadoxTalonFX launcherOneMotor;
    protected final PearadoxTalonFX launcherTwoMotor;

    protected LauncherIOTalonFX() {
        launcherOneMotor = new PearadoxTalonFX(LauncherConstants.LAUNCHER_ONE_ID, LauncherConstants.getLauncherConfig());
        launcherTwoMotor = new PearadoxTalonFX(LauncherConstants.LAUNCHER_TWO_ID, LauncherConstants.getLauncherConfig());

        launcherTwoMotor.setControl(new Follower(LauncherConstants.LAUNCHER_TWO_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.launcherOneData = launcherOneMotor.getData();
        inputs.launcherTwoData = launcherTwoMotor.getData();
    }

    @Override
    public void runVolts(double volts) {
        launcherOneMotor.setVoltage(volts);
    }
}
