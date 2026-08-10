package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.crystalcaverns.FieldConstants;

// for robor reveal
public class Cameras {    
    private static final LoggedTunableNumber x = new LoggedTunableNumber("Camera/x", FieldConstants.RED_CAVE_ORIGIN.getX());
    private static final LoggedTunableNumber y = new LoggedTunableNumber("Camera/y", FieldConstants.RED_CAVE_ORIGIN.getY());
    private static final LoggedTunableNumber z = new LoggedTunableNumber("Camera/z", 1.67);
    private static final LoggedTunableNumber roll = new LoggedTunableNumber("Camera/r", 0.0);
    private static final LoggedTunableNumber pitch = new LoggedTunableNumber("Camera/p", 30);
    private static final LoggedTunableNumber yaw = new LoggedTunableNumber("Camera/yaw", 0.0);


    public static void log(Pose2d roborPose) {
        Translation3d camera = new Translation3d(x.get(), y.get(), z.get());

        Translation2d diff = roborPose.getTranslation().minus(camera.toTranslation2d());

        Logger.recordOutput("FieldSimulation/Camera1", new Pose3d(
            camera, new Rotation3d(Math.toRadians(roll.get()), Math.toRadians(pitch.get()), Math.toRadians(yaw.get()) + diff.getAngle().getRadians())));
    }
}
