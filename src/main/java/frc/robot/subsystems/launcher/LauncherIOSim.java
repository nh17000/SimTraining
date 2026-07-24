package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class LauncherIOSim extends LauncherIOTalonFX {
    private final DCMotorSim rollerPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    LauncherConstants.LAUNCHER_MOTORS, LauncherConstants.LAUNCHER_MOI, LauncherConstants.LAUNCHER_GEAR_RATIO),
            LauncherConstants.LAUNCHER_MOTORS);

    private final TalonFXSimState shooterOneSimState;

    public LauncherIOSim() {
        shooterOneSimState = launcherOneMotor.getSimState();
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        super.updateInputs(inputs);

        shooterOneSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        rollerPhysicsSim.setInputVoltage(shooterOneSimState.getMotorVoltage());
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        shooterOneSimState.setRawRotorPosition(
                rollerPhysicsSim.getAngularPositionRad() / LauncherConstants.LAUNCHER_P_COEFFICIENT);
        shooterOneSimState.setRotorVelocity(
                rollerPhysicsSim.getAngularVelocityRadPerSec() / LauncherConstants.LAUNCHER_P_COEFFICIENT);
    }
}