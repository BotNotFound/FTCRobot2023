package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public final class Grabber extends ModuleBase {

    /**
     * The first servo controlling the grabber
     */
    private final Servo servo1;

    /**
     * The second servo controlling the grabber
     */
    private final Servo servo2;

    /**
     * The default name for the first grabber motor
     */
    public static final String SERVO1_DEFAULT_NAME = "Grabber Servo 1";

    /**
     * The default name for the second grabber motor
     */
    public static final String SERVO2_DEFAULT_NAME = "Grabber Servo 2";

    /**
     * The amount the servos should rotate when the grabber grabs
     */
    private static final double ACTIVE_SERVO_ROTATION_OFFSET = 0.4f;

    private boolean isGrabbing;

    /**
     * Is the grabber active?
     * @return whether or not the grabber is grabbing
     */
    public boolean isGrabbing() {
        return isGrabbing;
    }

    /**
     * Given an OpMode, initializes the module with the default motors (ones with the module's default motor name)
     * @param registrar The OpMode initializing the module
     * @exception InterruptedException The module was unable to locate the necessary hardware elements
     */
    public Grabber(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            this.servo1 = registrar.hardwareMap.get(Servo.class, SERVO1_DEFAULT_NAME);
            this.servo2 = registrar.hardwareMap.get(Servo.class, SERVO2_DEFAULT_NAME);
        }
            catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        isGrabbing = true; // release() only runs if isGrabbing isn't set to false
        release(); // in case the grabber is currently active, deactivate it
    }

    @Override
    public void cleanupModule() {

    }

    /**
     * Rotates the grabber by the specified amount
     * @param rotation The amount to rotate the grabber by
     */
    public void rotate(double rotation) {
        getTelemetry().addData("Rotating grabber by:", rotation);

        // rotate relative to current position to preserve grab state
        servo1.setPosition(servo1.getPosition() + rotation);
        servo2.setPosition(servo2.getPosition() - rotation);
    }

    /**
     * If the grabber isn't currently active, activate it
     */
    public void grab() {
        if (isGrabbing()) { return; }
        isGrabbing = true;

        getTelemetry().addLine("Grabbing grabber");

        // rotating servos in different directions to rotate the middle gear
        servo1.setPosition(servo1.getPosition() + ACTIVE_SERVO_ROTATION_OFFSET);
        servo2.setPosition(servo2.getPosition() + ACTIVE_SERVO_ROTATION_OFFSET);
    }

    /**
     * If the grabber is currently active, deactivate it
     */
    public void release() {
        if (!isGrabbing()) { return; }
        isGrabbing = false;

        getTelemetry().addLine("Releasing grabber");

        // rotating servos in different directions to rotate the middle gear
        servo1.setPosition(servo1.getPosition()  - ACTIVE_SERVO_ROTATION_OFFSET);
        servo2.setPosition(servo2.getPosition() - ACTIVE_SERVO_ROTATION_OFFSET);
    }

    /**
     * Toggles whether or not the grabber is active
     */
    public void toggleGrabState() {
        if (!isGrabbing) {
            grab();
        }
        else {
            release();
        }
    }
}
