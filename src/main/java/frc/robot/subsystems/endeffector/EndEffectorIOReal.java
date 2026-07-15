package frc.robot.subsystems.endeffector;

public class EndEffectorIOReal extends EndEffectorIOTalonFX {
    public EndEffectorIOReal() {}

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        super.updateInputs(inputs);

        // todo: implement
        inputs.hasCrystal = inputs.frontData.isDetected() || inputs.backData.isDetected();
        inputs.hasGemstone = inputs.topData.statorCurrent() > 30;
    }
}
