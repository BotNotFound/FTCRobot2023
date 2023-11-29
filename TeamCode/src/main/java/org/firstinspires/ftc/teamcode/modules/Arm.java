package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.location.PIDController;

public final class Arm extends LinearSlide {

    public static double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    public static double ONE_REVOLUTION_DEGREES = 360;

    private final DcMotorEx jointMotor;

    public static final String JOINT_MOTOR_NAME = "Joint Motor";
    
    public static abstract class Presets {
        public static final double FACING_GROUND = 0.0;
        
        public static final double FACING_BACKDROP = Math.PI / 6;

        public static final double IDLE = FACING_GROUND;
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
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void log() {
        getTelemetry().addData("[Arm] current rotation", getRotation());
    }

    public void rotateJoint(double rotation) {
        final int targetPosition = (int)Math.round(rotation / ONE_REVOLUTION_DEGREES * ENCODER_RESOLUTION);
        int currentPosition;

        final PIDController.PIDConfig config = new PIDController.PIDConfig(
            1,
                0,
                0,
                1,
                0.8
        );
        final PIDController.MovementInfo info = new PIDController.MovementInfo();

        ElapsedTime timer = new ElapsedTime();

        do {
            double deltaTime = timer.time();
            timer.reset();

            currentPosition = jointMotor.getCurrentPosition();
            jointMotor.setPower(PIDController.calcVelocity(
                    config,
                    currentPosition,
                    targetPosition,
                    info,
                    deltaTime
            ));
        } while (currentPosition != targetPosition);
    }

    public double getRotation() {
        return jointMotor.getCurrentPosition();
    }
}
