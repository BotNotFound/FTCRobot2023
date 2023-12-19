package org.firstinspires.ftc.teamcode.hardware;

public final class GearRatio {
    private final double ratio;

    /**
     * Constructs a gear ratio containing two gears
     * @param in The amount of teeth on the first gear
     * @param out The amount of teeth on the second gear
     */
    public GearRatio(int in, int out) {
        this((double)in / out);
    }

    private GearRatio(double ratio) {
        this.ratio = ratio;
    }

    public GearRatio into(GearRatio nextGear) {
        return new GearRatio(ratio * nextGear.ratio);
    }

    public double getRatio() {
        return ratio;
    }

    public double calculateEnd(double start) {
        return start * getRatio();
    }

    public double calculateStart(double end) {
        return end / getRatio();
    }
}
