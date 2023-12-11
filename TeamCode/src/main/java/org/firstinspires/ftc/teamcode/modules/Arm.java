package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Arm extends ModuleBase {
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

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
    private final DcMotor armMotor;

    /**
     * The name of the arm motor on the hardware map
     */
    public static final String ARM_MOTOR_NAME = "Arm Motor";

    /**
     * The servo that rotates the wrist
     */
    private final Servo wristServo;

    /**
     * The name of the wrist servo on the hardware map
     */
    public static final String WRIST_SERVO_NAME = "Wrist Servo";

    /**
     * The servo that opens/closes the flap
     */
    private final Servo flapServo;

    /**
     * The name of the flap servo on the hardware map
     */
    public static final String FLAP_SERVO_NAME = "Flap Servo";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(OpMode registrar) throws InterruptedException {
        super(registrar);
        armMotor = parent.hardwareMap.get(DcMotor.class, ARM_MOTOR_NAME);
        wristServo = parent.hardwareMap.get(Servo.class, WRIST_SERVO_NAME);
        flapServo = parent.hardwareMap.get(Servo.class, FLAP_SERVO_NAME);

        isFlapOpen = true;
        closeFlap();
    }

    /**
     * Rotates the arm to the specified rotation
     * @param rotation The target rotation
     * @param angleUnit The unit of rotation used
     * @param preserveWristRotation should the wrist rotate with the arm so that it is facing the same direction at the end of rotation?
     */
    public void rotateArmTo(double rotation, AngleUnit angleUnit, boolean preserveWristRotation) {
        rotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

        throw new RuntimeException("Not implemented!"); // TODO
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
        rotation = ANGLE_UNIT.fromUnit(angleUnit, rotation); // convert angle to our unit

        throw new RuntimeException("Not implemented!"); // TODO
    }

    /**
     * Opens the flap, if the flap is not already open
     */
    public void openFlap() {
        if (isFlapOpen) {
            return;
        }
        isFlapOpen = false;

        throw new RuntimeException("Not implemented!"); // TODO
    }

    /**
     * Closes the flap, if the flap is not already closed
     */
    public void closeFlap() {
        if (!isFlapOpen) {
            return;
        }
        isFlapOpen = true;

        throw new RuntimeException("Not implemented!");
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
