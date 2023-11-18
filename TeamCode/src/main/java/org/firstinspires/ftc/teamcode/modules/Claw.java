package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.atomic.AtomicBoolean;

public class Claw extends ModuleBase {
    private final Servo clawServo;

    private final AtomicBoolean isGrabbing;

    public static final String CLAW_SERVO_DEFAULT_NAME = "Claw Servo";

    /* TODO when claws get added to the robot, change these values to ones that actually make the
            claw grab and release */
    public static final double CLAW_SERVO_POSITION_GRABBING = 1;
    public static final double CLAW_SERVO_POSITION_RELEASED = 0;

    public Claw(OpMode registrar, String servoName) {
        super(registrar);
        clawServo = registrar.hardwareMap.get(Servo.class, servoName);
        clawServo.setPosition(CLAW_SERVO_POSITION_RELEASED);
        isGrabbing = new AtomicBoolean(false);
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Claw(OpMode registrar) {
        this(registrar, CLAW_SERVO_DEFAULT_NAME);
    }

    public void grab() {
        if (isGrabbing.compareAndSet(false, true)) {
            clawServo.setPosition(CLAW_SERVO_POSITION_GRABBING);
        }
    }

    public void release() {
        if (isGrabbing.compareAndSet(true, false)) {
            clawServo.setPosition(CLAW_SERVO_POSITION_RELEASED);
        }
    }

    public void toggleGrabState() {
        if (isGrabbing.get()) {
            release();
        }
        else {
            grab();
        }
    }

    @Override
    public void cleanupModule() {
        release();
    }
}
