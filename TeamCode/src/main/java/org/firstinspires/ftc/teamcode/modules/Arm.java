package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Arm extends LinearSlide {

    public static double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    public static double ONE_REVOLUTION_DEGREES = 360;

    private final DcMotorEx jointMotor;

    public static final String JOINT_MOTOR_NAME = "Joint Motor";

    public static final double JOINT_MOTOR_POWER = 0.3;
    
    public static abstract class Presets {
        public static final double READY_FOR_INTAKE = -175.0;
        
        public static final double READY_FOR_SCORE = -95.0;

        public static final double IDLE = -1.0;
    }

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            jointMotor = parent.hardwareMap.get(DcMotorEx.class, JOINT_MOTOR_NAME);
        }
        catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setTargetPosition(0);
        jointMotor.setPower(JOINT_MOTOR_POWER);
        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] current rotation", getRotation());
    }

    public double getRotation() {
        return (double)jointMotor.getCurrentPosition() * ONE_REVOLUTION_DEGREES / ENCODER_RESOLUTION;
    }

    public void rotateJoint(double rotation) {
        jointMotor.setTargetPosition((int)(rotation / ONE_REVOLUTION_DEGREES * ENCODER_RESOLUTION));
    }
}
