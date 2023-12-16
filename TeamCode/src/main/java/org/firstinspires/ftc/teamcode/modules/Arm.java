package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.location.PIDController;

public final class Arm extends ModuleBase {
    /**
     * One full rotation of the arm in encoder ticks.<br />
     * Taken from <a href="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-24mm-length-8mm-rex-shaft-117-rpm-3-3-5v-encoder/">GoBilda</a>
     */
    public static final int ENCODER_RESOLUTION = ((((1+(46/17))) * (1+(46/17))) * (1+(46/17)) * 28);

    /**
     * The unit of rotation used by default
     */
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    /**
     * One full rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public static final double ONE_ROTATION = ANGLE_UNIT.fromDegrees(360.0);

    public static final class ArmPresets extends Presets { // TODO these values are untested
        /**
         * Rotates the arm so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = 0.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 180.0;

        /**
         * Rotates the arm so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 135.0;
    }

    public static final class WristPresets extends Presets { // TODO these presets are untested
        /**
         * Rotates the wrist so that the robot can collect pixels
         */
        public static final double READY_TO_INTAKE = 30.0;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the floor behind the active intake
         */
        public static final double DEPOSIT_ON_FLOOR = 180.0;

        /**
         * Rotates the wrist so that the robot can deposit pixels on the backdrop behind the active intake
         */
        public static final double DEPOSIT_ON_BACKDROP = 90.0;
    }

    private boolean isFlapOpen;

    /**
     * The motor that rotates the arm
     */
    private final ConditionalHardwareDevice<DcMotor> armMotor;

    /**
     * The name of the arm motor on the hardware map
     */
    public static final String ARM_MOTOR_NAME = "Arm Motor";

    /**
     * The servo that rotates the wrist
     */
    private final ConditionalHardwareDevice<Servo> wristServo;

    /**
     * The name of the wrist servo on the hardware map
     */
    public static final String WRIST_SERVO_NAME = "Wrist Servo";

    /**
     * The servo that opens/closes the flap
     */
    private final ConditionalHardwareDevice<Servo> flapServo;

    /**
     * The name of the flap servo on the hardware map
     */
    public static final String FLAP_SERVO_NAME = "Flap Servo";

    @Config
    public static class ArmConfig {
        /**
         * The rotation of the arm when the robot initializes
         */
        public static double ROTATION_AT_POSITION_0 = AngleUnit.RADIANS.fromDegrees(0);
        /**
         * The feed forward coefficient for the PID controller
         */
        public static double FEED_FORWARD_COEFFICIENT = 0.0;
        /**
         * The configuration of the PID controller
         */
        public static PIDController.PIDConfig ARM_PID_CONFIG = new PIDController.PIDConfig(
                0.1,
                0,
                0,
                (currentPos, targetPos) -> Math.sin(
                        ((currentPos / ENCODER_RESOLUTION) * (Math.PI * 2)) // current rotation, according to the encoder
                                + ROTATION_AT_POSITION_0) // add the offset to get our actual rotation
                        * FEED_FORWARD_COEFFICIENT, // feed-forward coefficient
                0.01
        );
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(OpMode registrar) {
        super(registrar);
        armMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, ARM_MOTOR_NAME);
        wristServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, WRIST_SERVO_NAME);
        flapServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, FLAP_SERVO_NAME);

        armMotor.runIfAvailable((arm) -> {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        });

        isFlapOpen = true;
        closeFlap();
    }

    private static class ArmData {
        public boolean isDirty = true;
        public int targetPosition = 0;
    }
    private final ArmData armData = new ArmData();

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit, boolean preserveWristRotation) {
        if (preserveWristRotation) {
            throw new UnsupportedOperationException("Not Implemented!"); // TODO
        }

        // since encoder resolution is bad, I eyeballed it and we're going to use 7000 as 1 revolution in ticks until
        // a better number comes up
        armData.targetPosition = (int)(ANGLE_UNIT.fromUnit(angleUnit, rotation) / ONE_ROTATION * 7000 /* <-- Here is the 7000 */);

        armData.isDirty = true;
    }

    /**
     * Rotates the arm to the specified rotation, WITHOUT preserving wrist rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit) {
        rotateArmTo(rotation, angleUnit, false);
    }

    /**
     * Rotates the wrist to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     */
    public void rotateWristTo(double rotation, AngleUnit angleUnit) {
        wristServo.runIfAvailable((Servo wrist) -> {
            final double convertedRotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

            throw new RuntimeException("Not implemented!"); // TODO
        });
    }

    /**
     * Gets the rotation of the arm
     * @return The arm's rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getArmRotation() {
        return ((double) armMotor.requireDevice().getCurrentPosition() / ENCODER_RESOLUTION) * ONE_ROTATION;
    }

    /**
     * Gets the rotation of the wrist
     * @return the rotation in the unit specified by {@link #ANGLE_UNIT}
     */
    public double getWristRotation() {
        return wristServo.requireDevice().getPosition() * (ONE_ROTATION / 2); // servo can only rotate up to 180 degrees (1/2 of a full rotation)
    }

    /**
     * Opens the flap, if the flap is not already open
     */
    public void openFlap() {
        flapServo.runIfAvailable((Servo flap) -> {
            if (isFlapOpen) {
                return;
            }
            isFlapOpen = true;

            throw new RuntimeException("Not implemented!"); // TODO
        });
    }

    /**
     * Closes the flap, if the flap is not already closed
     */
    public void closeFlap() {
        flapServo.runIfAvailable((Servo flap) -> {
            if (!isFlapOpen) {
                return;
            }
            isFlapOpen = false;

            throw new RuntimeException("Not implemented!");
        });
    }

    /**
     * If the flap is open, close it.  Otherwise, open the flap
     */
    public void toggleFlap() {
        if (isFlapOpen) {
            closeFlap();
        }
        else {
            openFlap();
        }
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
