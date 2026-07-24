package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class VisionIOFake implements VisionIO {
    private final Supplier<Pose2d> poseSupplier;

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.poseObservations = new PoseObservation[] {
            new PoseObservation(
                RobotController.getFPGATime(), 
                new Pose3d(poseSupplier.get()), 
                0.01, 
                2, 
                0.1, 
                PoseObservationType.PHOTONVISION)
        };
        inputs.tagIds = new int[] { 1, 2 };
    }
}
