package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

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
     * Used convert from motor position (in ticks) to distance (in millimeters)
     */
    public static final double TICKS_TO_MM = ENCODER_RESOLUTION / (WHEEL_RADIUS * 360);

    public Odometry(@NonNull OpMode registrar) {
        super(registrar);
        hardwareDevices.executeIfAllAreAvailable(() -> {
            getFrontRightMecanumDriver().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getFrontLeftMecanumDriver().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getBackLeftMecanumDriver().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getBackRightMecanumDriver().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getFrontRightMecanumDriver().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            getFrontLeftMecanumDriver().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            getBackRightMecanumDriver().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            getBackLeftMecanumDriver().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    @Override
    public void log() {
        super.log();
        hardwareDevices.executeIfAllAreAvailable(() ->
            getTelemetry().addData("[Odometry] Current Position", getLocation())
        );
    }

    @Override
    public synchronized LocalizedMovement getLocation() throws LocatorException {
        if (!hardwareDevices.areAllDevicesAvailable()) {
            throw new LocatorException(this, "Module does not have the necessary hardware devices!");
        }

        final double frontLeftPos = getMotorPosition(getFrontLeftMecanumDriver());
        final double frontRightPos = getMotorPosition(getFrontRightMecanumDriver());
        final double backLeftPos = getMotorPosition(getBackLeftMecanumDriver());
        final double backRightPos = getMotorPosition(getBackLeftMecanumDriver());

        final double forwardDistance = ((frontLeftPos + frontRightPos + backLeftPos + backRightPos) / 4) * TICKS_TO_MM;
        final double strafeDistance = ((frontLeftPos + frontRightPos - backLeftPos - backRightPos) / 4) * TICKS_TO_MM;
        final double rotation = getIMU().getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT);

        return new LocalizedMovement(forwardDistance, strafeDistance, rotation, this);
    }

    private static double getMotorPosition(DcMotor motor) {
        switch (motor.getDirection()) {
            case FORWARD:
                return motor.getCurrentPosition();
            case REVERSE:
                return -motor.getCurrentPosition();
            default:
                // there is no reason for anyone to make a new member for the Direction enum, so if we get here, just panic
                throw new IllegalStateException("HOW");
        }
    }

    @Override
    public boolean isActive() {
        return hardwareDevices.areAllDevicesAvailable();
    }

    @Override
    public LocatorKind getKind() {
        return LocatorKind.ROBOT_START_RELATIVE;
    }

    @Override
    public Movement getFieldSize() {
        return new Movement(3657.6, 3657.6, ANGLE_UNIT.getUnnormalized().fromDegrees(360));
    }
}
