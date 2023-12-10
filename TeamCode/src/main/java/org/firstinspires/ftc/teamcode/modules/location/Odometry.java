package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

import java.util.concurrent.TimeUnit;

public class Odometry extends FieldCentricDriveTrain implements Locator {
    /**
     * Whether rotation provided by {@see #getLocation()} are in degrees or radians
     */
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    /**
     * The radius of the mecanum wheels in millimeters
     */
    public static final double WHEEL_RADIUS = 48;

    /**
     * The amount of encoder ticks in one full revolution
     */
    public static final double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    /**
     * Used convert from motor position (in ticks) to distance (in milimeters)
     */
    public static final double TICKS_TO_MM = ENCODER_RESOLUTION / (WHEEL_RADIUS * 360);

    public Odometry(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
        frontRightMecanumDriver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMecanumDriver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMecanumDriver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMecanumDriver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMecanumDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void log() {
        super.log();
        getTelemetry().addData("[Odometry] Current Position", getLocation());
    }

    @Override
    public synchronized LocalizedMovement getLocation() throws LocatorException {
	final double frontLeftPos = frontLeftMecanumDriver.getCurrentPosition();
	final double frontRightPos = frontRightMecanumDriver.getCurrentPosition();
	final double backLeftPos = backLeftMecanumDriver.getCurrentPosition();
	final double backRightPos = backLeftMecanumDriver.getCurrentPosition();
	
	final double fowardDistance = ((frontLeftPos + frontRightPos + backLeftPos + backRightPos) / 4) * TICKS_TO_MM;
	final double strafeDistance = ((frontLeftPos + frontRightPos - backLeftPos - backRightPos) / 4) * TICKS_TO_MM;
	final double rotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

	return new LocalizedMovement(fowardDistance, strafeDistance, rotation, this);
    }

    @Override
    public boolean isActive() {
        return true;
    }

    @Override
    public LocatorKind getKind() {
        return LocatorKind.ROBOT_START_RELATIVE;
    }

    @Override
    public Movement getFieldSize() {
        return new Movement(3657.6, 3657.6, 360);
    }
}
