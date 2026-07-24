package frc.robot.subsystems.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.LauncherConstants.LauncherState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
    @AutoLogOutput
    @Getter
    private LauncherState state = LauncherState.OFF;

    private final LoggedTunableNumber manualVoltage = new LoggedTunableNumber("Launcher/volts", 4.5);
    private final InterpolatingDoubleTreeMap rpsLerp = new InterpolatingDoubleTreeMap();

    private LauncherIO io;
    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    @Setter
    private DoubleSupplier distanceToPassingTargetSupplier = () -> 0.0;

    public Launcher(LauncherIO io) {
        this.io = io;

        rpsLerp.put(14.0, 4.5);
        rpsLerp.put(11.0, 4.0);
        rpsLerp.put(9.0, 3.5);
        rpsLerp.put(5.0, 3.0);
        rpsLerp.put(2.0, 1.5);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);

        double desiredVolts = switch (state) {
            case SELF_DIRECTING -> rpsLerp.get(distanceToPassingTargetSupplier.getAsDouble());
            case MANUAL -> manualVoltage.get();
            default -> 0.0;
        };

        io.runVolts(desiredVolts);

        Logger.recordOutput("Launcher/Desired Voltage", desiredVolts);
    }

    public Command setSelfDirecting() {
        return runOnce(() -> state = LauncherState.SELF_DIRECTING);
    }

    public Command setManual() {
        return runOnce(() -> state = LauncherState.MANUAL);
    }

    public Command setOff() {
        return runOnce(() -> state = LauncherState.OFF);
    }

    @AutoLogOutput
    public double getAngularPositionRads() {
        return inputs.launcherOneData.position() * LauncherConstants.LAUNCHER_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getAngularVelocityRadPerSec() {
        return inputs.launcherOneData.velocity() * LauncherConstants.LAUNCHER_P_COEFFICIENT;
    }
}