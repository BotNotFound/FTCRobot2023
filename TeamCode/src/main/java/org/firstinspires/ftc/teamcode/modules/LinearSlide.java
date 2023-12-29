package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.modules.core.Module;

@Deprecated
public class LinearSlide extends Module {
    /**
     * The slide motor
     */
    private final DcMotor slideMotor;

    /**
     * The name of the slide motor
     */
    public static final String SLIDE_MOTOR_NAME = "Linear Slide Motor";

    /**
     * The height of the linear slide at full extension, in millimeters
     */
    public static final double SLIDE_HEIGHT_MAX_EXTENSION = 540;

    /**
     * The diameter of the motor spool, in millimeters
     */
    public static final double MOTOR_DIAMETER = 35;

    /**
     * The rotation the motor will be at when the linear slide is fully extended, in radians
     */
    public static final double RADIANS_MAX_EXTENSION = SLIDE_HEIGHT_MAX_EXTENSION / (MOTOR_DIAMETER / 2);

    /**
     * One full rotation of the motor, in ticks. <br />
     * Taken from
     * <a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/">GoBuilda</a>
     */
    public static final int ENCODER_RESOLUTION = ((((1+(46/17))) * (1+(46/11))) * 28);

    /**
     * Presets for linear slide extension
     * @see #extendTo(double)
     */
    public static final class SlidePresets extends Presets {
        /**
         * Fully retracts the linear slides
         */
        public static final double NO_EXTENSION = 0;

        /**
         * Fully extends one of the three linear slides
         */
        public static final double ONE_SLIDE_USED = 0.3;

        /**
         * Fully extends two of the three linear slides
         */
        public static final double TWO_SLIDES_USED = 0.6;

        /**
         * Fully extends the linear slides
         */
        public static final double FULL_EXTENSION = 1;
        
        public static final double IDLE = NO_EXTENSION;
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public LinearSlide(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            slideMotor = parent.hardwareMap.get(DcMotor.class, SLIDE_MOTOR_NAME);
        }
        catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Extends the linear slide to the specified height
     * @param height A value in the range [0,1], where 0 fully retracts the linear slide and 1 fully extends it
     * @see SlidePresets
     */
    public void extendTo(double height) {
        final double ONE_REVOLUTION_RADIANS = 2 * Math.PI;

        double targetRotation_radians = height * RADIANS_MAX_EXTENSION;
        double targetRotation_revolutions = targetRotation_radians / ONE_REVOLUTION_RADIANS;
        long targetRotation_ticks = Math.round(targetRotation_revolutions * ENCODER_RESOLUTION);
        slideMotor.setTargetPosition((int)targetRotation_ticks); // double rounds to long, but setTargetPosition takes an int
    }

    public double getCurrentExtension() {
        return slideMotor.getCurrentPosition();
    }

    /**
     * Cleans up the module
     */
    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {
        getTelemetry().addData("[Linear Slides] Current extension", getCurrentExtension());
    }
}
