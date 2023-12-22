package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@Autonomous(group = "Tests")
public class DriverMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor frontLeftMotor = hardwareMap.get(
                DcMotor.class, DriveTrain.FRONT_LEFT_MECANUM_DRIVER_DEFAULT_NAME),
                frontRightMotor = hardwareMap.get(DcMotor.class,
                        DriveTrain.FRONT_RIGHT_MECANUM_DRIVER_DEFAULT_NAME),
                backLeftMotor = hardwareMap.get(DcMotor.class,
                        DriveTrain.BACK_LEFT_MECANUM_DRIVER_DEFAULT_NAME),
                backRightMotor = hardwareMap.get(DcMotor.class,
                        DriveTrain.BACK_RIGHT_MECANUM_DRIVER_DEFAULT_NAME);

        waitForStart();

        telemetry.addLine("moving front left...");
        telemetry.update();
        frontLeftMotor.setPower(0.25);
        sleep(1000);
        frontLeftMotor.setPower(0);

        telemetry.addLine("moving front right...");
        telemetry.update();
        frontRightMotor.setPower(0.25);
        sleep(1000);
        frontRightMotor.setPower(0);

        telemetry.addLine("moving back left...");
        telemetry.update();
        backLeftMotor.setPower(0.25);
        sleep(1000);
        backLeftMotor.setPower(0);

        telemetry.addLine("moving back right...");
        telemetry.update();
        backRightMotor.setPower(0.25);
        sleep(1000);
        backRightMotor.setPower(0);
    }
}
