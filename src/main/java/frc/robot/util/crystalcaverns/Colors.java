package frc.robot.util.crystalcaverns;

public class Colors {
    public enum CrystalColor {        
        Green(0),
        Yellow(1),
        Orange(2),
        Pearple(3),
        Camera(4);
        
        public final int index;
        private CrystalColor(int i) {
            this.index = i;
        }
    }

    public enum GemstoneColor {
        Red,
        Blue
    }

    public static CrystalColor getRandomCrystalColor() {
        return switch ((int) (Math.random() * 4)) {
            case 0 -> CrystalColor.Green;
            case 1 -> CrystalColor.Yellow;
            case 2 -> CrystalColor.Orange;
            default -> CrystalColor.Pearple;
        };
    }
}
