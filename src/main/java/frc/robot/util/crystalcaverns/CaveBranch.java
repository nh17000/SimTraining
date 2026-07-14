package frc.robot.util.crystalcaverns;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.Goal;

/**
 *
 *
 * <h2>Simulates a branch on the field.</h2>
 *
 * <p>This class simulates a branch where crystals can be scored. This class should not be used directly and
 * instead should be used via a {@link CaveSim} which will handle an entire cave.
 */
public class CaveBranch extends Goal {

    public final int level; // 0 or 1
    public final int column; // 0 to 7

    private static Angle angleTolerance = Degrees.of(99999); // temp 
    private static final List<CaveBranch> allInstances = new ArrayList<>();

    /**
     *
     *
     * <h2>Sets the Angle Tolerance for All cave Branches</h2>
     *
     * <p>This configures the tolerance used for rotation validation across all cave branch instances. Changes take
     * effect immediately for all instances, including those already created.
     *
     * @param tolerance The angle tolerance to use for all branches
     */
    public static void setAngleTolerance(Angle tolerance) {
        angleTolerance = tolerance;
        // Update all existing instances
        for (CaveBranch branch : allInstances) branch.updateRotationChecker();
    }

    // public static final Translation2d origin =
    //         new Translation2d(FieldMirroringUtils.FIELD_WIDTH / 2, FieldMirroringUtils.FIELD_HEIGHT / 2);
    // public static final Translation2d[] branchesCenterPositionBlue = new Translation2d[] {
    //     new Translation2d(-4.810, 0.164).plus(origin), // A
    //     new Translation2d(-4.810, -0.164).plus(origin), // B
    //     new Translation2d(-4.690, -0.373).plus(origin), // C
    //     new Translation2d(-4.406, -0.538).plus(origin), // D
    //     new Translation2d(-4.164, -0.537).plus(origin), // E
    //     new Translation2d(-3.879, -0.374).plus(origin), // F
    //     new Translation2d(-3.759, -0.164).plus(origin), // G
    //     new Translation2d(-3.759, 0.164).plus(origin), // H
    //     new Translation2d(-3.880, 0.373).plus(origin), // I
    //     new Translation2d(-4.164, 0.538).plus(origin), // J
    //     new Translation2d(-4.405, 0.538).plus(origin), // K
    //     new Translation2d(-4.690, 0.374).plus(origin) // L
    // };

    // public static final Translation3d[] heights = new Translation3d[] {
    //     new Translation3d(0, 0, 0.45),
    //     new Translation3d(0, 0, 0.79),
    //     new Translation3d(0, 0, 1.19),
    //     new Translation3d(0, 0, 1.78)
    // };

    // public static final Rotation3d flip90 = new Rotation3d(0, 0, Math.PI / 2);

    // public static final Translation2d[] branchesCenterPositionRed = Arrays.stream(branchesCenterPositionBlue)
    //         .map(FieldMirroringUtils::flip)
    //         .toArray(Translation2d[]::new);

    // public static final Rotation2d[] branchesFacingOutwardsBlue = new Rotation2d[] {
    //     Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180), // A and B
    //     Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(240), // C and D
    //     Rotation2d.fromDegrees(300), Rotation2d.fromDegrees(300), // E and F
    //     Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), // G and H
    //     Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(60), // I and J
    //     Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(120), // K and L
    // };

    /**
     *
     *
     * <h2>Returns the required pose of a cave branch at the designated position.</h2>
     *
     * @param isBlue Wether the position is on the blue cave or the red cave.
     * @param level The level of the cave (0 indexed). Range of 0-3.
     * @param col The pole or Colum of the cave (0 indexed). Range of 0-11.
     * @return The pose of a cave branch with the specified stats.
     */
    public static Translation3d getPoseOfBranchAt(boolean isBlue, int level, int col) {
        // Translation3d branchCenterPosition = isBlue
        //         ? new Translation3d(FieldConstants.BLUE_LOWER_BRANCHES[col])
        //         : new Translation3d(FieldConstants.RED_LOWER_BRANCHES[col]);

        // return branchCenterPosition
        //         // .plus(level > 0 ? FieldConstants.UPPER_BRANCH_HEIGHT : FieldConstants.LOWER_BRANCH_HEIGHT);
        //         // .plus(new Translation3d(0.255, 0, 0).rotateBy(new Rotation3d(branchesFacingOutwardsBlue[col])));
        //         .plus(new Translation3d(0.255, 0, level > 0 ? FieldConstants.UPPER_BRANCH_HEIGHT : FieldConstants.LOWER_BRANCH_HEIGHT));

        Translation2d temp;
        double h;
        if (level == 0) {
            temp = isBlue ? FieldConstants.BLUE_LOWER_BRANCHES[col] : FieldConstants.RED_LOWER_BRANCHES[col];
            h = FieldConstants.LOWER_BRANCH_HEIGHT;
        } else {
            temp = isBlue ? FieldConstants.BLUE_UPPER_BRANCHES[col] : FieldConstants.RED_UPPER_BRANCHES[col];
            h = FieldConstants.UPPER_BRANCH_HEIGHT;
        }
        return new Translation3d(temp.getX(), temp.getY(), h);
    }

    // public static final Rotation2d[] branchesFacingOutwardsRed = Arrays.stream(branchesFacingOutwardsBlue)
    //         .map(FieldMirroringUtils::flip)
    //         .toArray(Rotation2d[]::new);

    private static final int MIN_LEVEL = 0;
    private static final int MAX_LEVEL = 1;

    /**
     *
     *
     * <h2>Creates a singular cave branch at the specified location </h2>
     *
     * @param arena The host arena of this cave.
     * @param isBlue Wether the position is on the blue cave or the red cave.
     * @param level The level of the cave (0 indexed). Range of 0-1.
     * @param column The pole or Colum of the cave (0 indexed). Range of 0-7.
     */
    public CaveBranch(CrystalCavernsArena arena, boolean isBlue, int level, int column) {
        super(
                arena,
                Inches.of(11),
                Inches.of(11),
                Inches.of(11),
                "Crystal",
                getPoseOfBranchAt(isBlue, level, column),
                isBlue,
                // level == 0 ? 2 : 1,
                4,
                false);

        if (level < MIN_LEVEL || level > MAX_LEVEL) {
            throw new IllegalArgumentException(
                    "Invalid cave level: " + level + " (must be between " + MIN_LEVEL + " and " + MAX_LEVEL + ")");
        }

        this.level = level;
        this.column = column;

        // Set initial rotation checker
        updateRotationChecker();

        // Register this instance for future tolerance updates
        allInstances.add(this);
    }

    /**
     *
     *
     * <h2>Updates the Rotation Checker Based on Current Angle Tolerance</h2>
     *
     * <p>This method is called during construction and whenever the static angle tolerance is changed.
     */
    private void updateRotationChecker() {
        switch (level) {
            case 0, 1 -> {
                Rotation3d targetAngle = new Rotation3d(
                        Degrees.zero(),
                        Degrees.zero(),
                        Degrees.zero());
                setNeededAngle(targetAngle, angleTolerance);
            }
            // case 3 -> setNeededAngle(new Rotation3d(Degrees.zero(), Degrees.of(90), Degrees.zero()), angleTolerance);
        } // case 0 uses default anyRotation() from parent
    }

    /**
     *
     *
     * <h2>Gives the pose of the cave branch.</h2>
     *
     * @return This position of this branch as a pose3d.
     */
    public Pose3d getPose() {
        Rotation3d facing = new Rotation3d(
                0,
                // level == 1 || level == 2 ? -35 * Math.PI / 180 : 0,
                0,
                //(isBlue ? branchesFacingOutwardsBlue : branchesFacingOutwardsRed)[column].getRadians()
                0
                );

        // if (level == 0) {
        //     facing = facing.plus(flip90);
        // }

        return new Pose3d(position, facing);
    }

    private static final int[] AUTO_POINTS_BY_LEVEL = {2, 3};
    private static final int[] TELEOP_POINTS_BY_LEVEL = {3, 4};

    @Override
    protected void addPoints() {
        System.out.println("Crystal scored on level: " + (level + 1) + " on the " + (isBlue ? "Blue " : "Red") + "cave");
        arena.addValueToMatchBreakdown(isBlue, "Auto/CrystalScoredInAuto", DriverStation.isAutonomous() ? 1 : 0);
        arena.addValueToMatchBreakdown(isBlue, "CrystalScoredOnLevel " + String.valueOf(level + 1), 1);

        int points = DriverStation.isAutonomous() ? AUTO_POINTS_BY_LEVEL[level] : TELEOP_POINTS_BY_LEVEL[level];
        arena.addToScore(isBlue, points);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
        for (int i = 0; i < this.gamePieceCount; i++) {
            drawList.add(getPose().transformBy(new Transform3d(0, 0, Inches.of(2 * i + 1).in(Meters), Rotation3d.kZero)));
        }
    }
}