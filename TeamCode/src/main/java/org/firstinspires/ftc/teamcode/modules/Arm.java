package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;

public final class Arm extends LinearSlide {

    public static double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    public static double ONE_REVOLUTION_RADIANS = 2 * Math.PI;

    private final DcMotorEx jointMotor;

    public static final String JOINT_MOTOR_NAME = "Joint Motor";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public Arm(@NonNull @NotNull OpMode registrar) throws InterruptedException {
        super(registrar);
        try {
            jointMotor = parent.hardwareMap.get(DcMotorEx.class, JOINT_MOTOR_NAME);
        }
        catch (IllegalArgumentException e) {
            throw new InterruptedException(e.getMessage());
        }

        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setTargetPosition(0);
        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateJoint(double rotation) {
        jointMotor.setTargetPosition((int)Math.round(rotation / ONE_REVOLUTION_RADIANS * ENCODER_RESOLUTION));
    }
}
