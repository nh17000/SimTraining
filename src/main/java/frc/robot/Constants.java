// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.crystalcaverns.FieldConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double LOOP_PERIOD = 0.02; // 20ms
    public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
    public static final double NOMINAL_VOLTAGE = 12;

    public static final class AlignConstants {
        public static final double BRANCH_SPACING = Units.inchesToMeters(5);
        // public static final double REEF_ALIGN_TZ = 0; // Units.inchesToMeters(30);

        public static final Translation2d LEFT_BRANCH_OFFSET = new Translation2d(0.0, -BRANCH_SPACING);
        public static final Translation2d RIGHT_BRANCH_OFFSET = new Translation2d(0.0, BRANCH_SPACING);
        public static final Translation2d MID_OFFSET = new Translation2d(0.0, 0.0);
        // public static final Translation2d STATION_OFFSET = new Translation2d(Units.inchesToMeters(18), 0.0);

        public static final Transform2d MINE_TRANSFORM = new Transform2d(Units.inchesToMeters(28), 0, Rotation2d.k180deg);
        public static final Translation2d BACKWARDS_OFFSET = new Translation2d(Units.inchesToMeters(12.5), 0);

        public static final double DRIVE_kP = 5.0; // m/s per m error
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double MAX_DRIVE_VELOCITY = 3.0; // m/s
        public static final double MAX_DRIVE_ACCELERATION = 10; // m/s^2

        public static final double ROT_kP = 5.0; // rads per rad error
        public static final double ROT_kI = 0.0;
        public static final double ROT_kD = 0.0;
        public static final double MAX_ROT_VELOCITY = 8; // rad/s
        public static final double MAX_ROT_ACCELERATION = 20; // rad/s^2

        public static final double ALIGN_ROT_TOLERANCE = Units.degreesToRadians(3);
        public static final double ALIGN_TRANSLATION_TOLERANCE = Units.inchesToMeters(2);

        public static final double NET_ALIGN_TZ = Units.inchesToMeters(32);
        public static final double MIN_DIST_TO_WALL = Units.inchesToMeters(28);
        public static final double[] NET_RED_Y = {MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH / 2.0 - MIN_DIST_TO_WALL
        };
        public static final double[] NET_BLUE_Y = {
            FieldConstants.FIELD_WIDTH / 2.0 + MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH - MIN_DIST_TO_WALL
        };
    }

    public static final class VisualizerConstants {
        public static final Translation3d STAGE0_ZERO = new Translation3d(-0.241300, 0, 0.377825);
        public static final Translation3d WRIST_ZERO = new Translation3d(0.535083, 0, 0.238125);
        public static final Translation3d WRIST_OFFSET = WRIST_ZERO.minus(STAGE0_ZERO);

        public static final Rotation2d CLIMBER_ANGLE = Rotation2d.fromDegrees(34);

        public static final Transform3d CRYSTAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(5.25), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(45), 0)),
                new Rotation3d(0, -Units.degreesToRadians(125), 0));

        public static final Transform3d HORIZONTAL_CORAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.5), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(96), 0)),
                new Rotation3d(0, 0, Units.degreesToRadians(90)));

        public static final Transform3d GEMSTONE_TRANSFORM = new Transform3d(
                new Translation3d(0, 0, 0.33)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(5), 0)),
                new Rotation3d(0, -Units.degreesToRadians(125), 0));
    }
}
