package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Arm extends ModuleBase {

    /**
     * The encoder resolution -- amount of 'rotation' equal to 1 full revolution of the motor
     */
    public static final int ENCODER_RESOLUTION = ((((1+(46/17))) * (1+(46/17))) * (1+(46/17)) * 28);

    /**
     * The motor moving the arm
     */
    private final DcMotor armMotor;

    /**
     * Default name for the armMotor
     */
    public static final String ARM_MOTOR_DEFAULT_NAME = "Arm Motor";


    /**
     * Given an OpMode, initializes the module with the default motor (one with the module's default motor name)
     * @param registrar The OpMode initializing the module
     * @exception InterruptedException The module was unable to locate the necessary motors
     */
    public Arm(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            this.armMotor = registrar.hardwareMap.get(DcMotor.class, ARM_MOTOR_DEFAULT_NAME);
        }
        catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        // motor config
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void cleanupModule() {

    }

    public void setRotation(int rotation) {
        getTelemetry().addData("Rotating arm to:", rotation);

        armMotor.setTargetPosition(rotation);
    }

    public int getRotation() {
        return armMotor.getTargetPosition();
    }

    public boolean isRotating() {
        return armMotor.getCurrentPosition() == armMotor.getTargetPosition();
    }
}