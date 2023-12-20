package org.firstinspires.ftc.teamcode.modules.detection;

public enum Prop {
    RED_TEAM_PROP((byte) 255, (byte) 255, (byte) 0, (byte) 0),
    BLUE_TEAM_PROP((byte) 255, (byte) 50, (byte) 50, (byte) 180);

    /**
     * How many bits is the alpha channel shifted?
     * Can also be thought of as the starting bit of the channel
     */
    public static final int ALPHA_SHIFT = 24;

    /**
     * How many bits is the red channel shifted?
     * Can also be thought of as the starting bit of the channel
     */
    public static final int RED_SHIFT = 16;

    /**
     * How many bits is the green channel shifted?
     * Can also be thought of as the starting bit of the channel
     */
    public static final int GREEN_SHIFT = 8;

    /**
     * How many bits is the blue channel shifted?
     * Can also be thought of as the starting bit of the channel
     */
    public static final int BLUE_SHIFT = 0;

    /**
     * A 32-bit integer containing the alpha, red, green, and blue channels
     */
    public final int argb;

    /**
     * @return The alpha channel of the prop's color
     */
    public byte alphaChannel() {
        return (byte)(argb >> ALPHA_SHIFT);
    }

    /**
     * @return The red channel of the prop's color
     */
    public byte redChannel() {
        return (byte)(argb >> RED_SHIFT);
    }

    /**
     * @return The green channel of the prop's color
     */
    public byte greenChannel() {
        return (byte)(argb >> GREEN_SHIFT);
    }

    /**
     * @return The blue channel of the prop's color
     */
    public byte blueChannel() {
        return (byte)(argb >> BLUE_SHIFT);
    }

    Prop(byte a, byte r, byte g, byte b) {
        this.argb = (a << ALPHA_SHIFT) | (r << RED_SHIFT) | (g << GREEN_SHIFT) | (b << BLUE_SHIFT);
    }
}
