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
    public static final double DEFAULT_GRABBING_SERVO_POSITION = 1.0;
    public static final double DEFAULT_RELEASED_SERVO_POSITION = 0.5;

    public final double grabbingServoPosition;
    public final double releasedServoPosition;

    public Claw(OpMode registrar, String servoName, double grabbingServoPosition, double releasedServoPosition) {
        super(registrar);
        getTelemetry().addLine("[" + servoName +"] grab pos: " + grabbingServoPosition);
        getTelemetry().addLine("[" + servoName +"] release pos: " + releasedServoPosition);
        this.grabbingServoPosition = (grabbingServoPosition % 1 == 0 && grabbingServoPosition != 0) ? 1 : (1 + grabbingServoPosition) % 1;
        this.releasedServoPosition = (releasedServoPosition % 1 == 0 && releasedServoPosition != 0) ? 1 : (1 + releasedServoPosition) % 1;
        clawServo = registrar.hardwareMap.get(Servo.class, servoName);
        clawServo.setPosition(this.releasedServoPosition);
        isGrabbing = new AtomicBoolean(false);
    }

    public Claw(OpMode registrar, String servoName) {
        this(registrar, servoName, DEFAULT_GRABBING_SERVO_POSITION, DEFAULT_RELEASED_SERVO_POSITION);
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Claw(OpMode registrar) {
        this(registrar, CLAW_SERVO_DEFAULT_NAME);
    }

    public boolean getGrabState() {
        return isGrabbing.get();
    }

    public void grab() {
        if (isGrabbing.compareAndSet(false, true)) {
            clawServo.setPosition(grabbingServoPosition);
        }
    }

    public void release() {
        if (isGrabbing.compareAndSet(true, false)) {
            clawServo.setPosition(releasedServoPosition);
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
    public void log() {
        getTelemetry().addData(clawServo.getDeviceName() + " pos", clawServo.getPosition());
    }

    @Override
    public void cleanupModule() {
        release();
    }
}
