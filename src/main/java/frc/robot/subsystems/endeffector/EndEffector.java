package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEState;
import frc.robot.subsystems.endeffector.EndEffectorConstants.GamePieceMode;

import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    @AutoLogOutput
    @Getter
    private EEState state = EEState.OFF;

    @AutoLogOutput
    @Getter
    @Setter
    private GamePieceMode mode = GamePieceMode.NONCLASSIFIER;

    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);

        io.setLeftVolts(state.leftVolts);
        io.setRightVolts(state.rightVolts);
        io.setTopVolts(state.topVolts);
    }

    public Command intake() {
        return run(() -> {
            boolean frontDetected = inputs.frontData.isDetected();
            boolean backDetected = inputs.backData.isDetected();

            if (hasGemstone() || (frontDetected && backDetected)) state = EEState.OFF;
            else if (frontDetected) state = EEState.INDEXING_FWD;
            else if (backDetected) state = EEState.INDEXING_BWD;
            else state = EEState.INTAKE;
        });
    }

    public Command index(BooleanSupplier isForwardsSupplier) {
        return run(() -> {
            boolean isForwards = isForwardsSupplier.getAsBoolean();
            boolean frontTripped = inputs.frontData.isDetected();
            boolean backTripped = inputs.backData.isDetected();

            if (hasGemstone()) state = EEState.OFF;
            else if (isForwards && backTripped) state = EEState.INDEXING_BWD;
            else if (!isForwards && frontTripped) state = EEState.INDEXING_FWD;
            else state = EEState.OFF;
        });
    }

    public Command outtake() {
        return run(() -> {
            state = EEState.OUTTAKE;
        });
    }

    public Command outtakeClassifier() {
        return run(() -> {
            state = EEState.OUTTAKE_CLASSIFIER;
        });
    }

    public boolean hasCrystal() {
        return inputs.hasCrystal;
    }

    public boolean hasGemstone() {
        return inputs.hasGemstone;
    }

    public boolean hasGamePiece() {
        return hasCrystal() || hasGemstone();
    }
}
