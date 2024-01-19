package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;

public class FieldCentricDriveTrain extends DriveTrain {

    public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    /**
     * The IMU
     *
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final IMU getIMU() {
        return hardwareDevices.getLoadedDevice(IMU.class, IMU_NAME);
    }

    private static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

    public static IMU.Parameters getImuParameters() {
        return IMU_PARAMETERS;
    }

    public static final String IMU_NAME = "imu";

    public FieldCentricDriveTrain(OpMode registrar) {
        super(registrar);
        hardwareDevices.tryLoadDevice(IMU.class, IMU_NAME);

        hardwareDevices.executeIfAllAreAvailable(() -> {
            getIMU().initialize(getImuParameters());
            resetRotation();
            getTelemetry().addLine("[Field Centric Drive Train] Found IMU");
        }, () -> getTelemetry().addLine("[Field Centric Drive Train] Couldn't find IMU!"));
    }

    public void resetRotation() {
        hardwareDevices.executeIfAllAreAvailable(getIMU()::resetYaw);
    }

    @Override
//    public void setVelocity(double distX, double distY, double rotation) {
//        hardwareDevices.executeIfAllAreAvailable(() -> {
//            double botHeading = getIMU().getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT)/* - curZero*/;
//
//            // Rotate the movement direction counter to the robot's rotation
//            double rotX = (distX * Math.cos(-botHeading) - distY * Math.sin(-botHeading));
//            double rotY = distX * Math.sin(-botHeading) + distY * Math.cos(-botHeading);
//            getTelemetry().addData("[Field Centric Drive Train] current x rotation", rotX);
//            getTelemetry().addData("[Field Centric Drive Train] current y rotation", rotY);
//            getTelemetry().addData("[Field Centric Drive Train] bot heading value", botHeading);
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
//            double frontLeftPower = (rotY + rotX + rotation) / denominator;
//            double backLeftPower = (rotY - rotX + rotation) / denominator;
//            double frontRightPower = (rotY - rotX - rotation) / denominator;
//            double backRightPower = (rotY + rotX - rotation) / denominator;
//            getTelemetry().addData("X stick", distX);
//            getTelemetry().addData("Y stick", distY);
//            getTelemetry().addData("Rotation", rotation);
//            getTelemetry().update();
//            //Set power to motors
//            getFrontLeftMecanumDriver().setPower(frontLeftPower);
//            getBackLeftMecanumDriver().setPower(backLeftPower);
//            getFrontRightMecanumDriver().setPower(frontRightPower);
//            getBackRightMecanumDriver().setPower(backRightPower);
//        });
//   }
    public void setVelocity(double distX, double distY, double rotation) {
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            double botHeading = getIMU().getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = distX * Math.cos(-botHeading) - distY * Math.sin(-botHeading);
            double rotY = distX * Math.sin(-botHeading) + distY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
            double frontLeftPower = (rotY + rotX + rotation) / denominator;
            double backLeftPower = (rotY - rotX + rotation) / denominator;
            double frontRightPower = (rotY + rotX - rotation) / denominator;
            double backRightPower = (rotY - rotX - rotation) / denominator;

            getFrontLeftMecanumDriver().setPower(frontLeftPower);
            getBackLeftMecanumDriver().setPower(backLeftPower);
            getFrontRightMecanumDriver().setPower(frontRightPower);
            getBackRightMecanumDriver().setPower(backRightPower);
    }
}
