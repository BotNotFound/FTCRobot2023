package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDriveTrain extends DriveTrain {

    public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    protected final IMU imu;

    public static final String IMU_NAME = "imu";

    public FieldCentricDriveTrain(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        imu = registrar.hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        resetRotation();
    }

    public void resetRotation() {
        imu.resetYaw();
    }

    @Override
    public void setVelocity(double distX, double distY, double rotation) {
        // negate these values so the algorithm works correctly
        distX = -distX;
        distY = -distY;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT)/* - curZero*/;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = -(distX * Math.cos(-botHeading) - distY * Math.sin(-botHeading));
        double rotY = distX * Math.sin(-botHeading) + distY * Math.cos(-botHeading);
        getTelemetry().addData("current x rotation", rotX);
        getTelemetry().addData("current y rotation", rotY);
        getTelemetry().addData("bot heading value", botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (-rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (-rotY + rotX - rotation) / denominator;
        //Set power to motors
        frontLeftMecanumDriver.setPower(frontLeftPower);
        backLeftMecanumDriver.setPower(backLeftPower);
        frontRightMecanumDriver.setPower(frontRightPower);
        backRightMecanumDriver.setPower(backRightPower);
    }
}
