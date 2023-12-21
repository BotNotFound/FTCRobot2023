package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.annotations.ModuleInitializer;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

public class Odometry extends FieldCentricDriveTrain implements Locator {
    /**
     * Whether rotation provided by {@see #getLocation()} are in degrees or radians
     */
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    /**
     * The diameter of the mecanum wheels in millimeters. <br />
     *  Taken from <a href="https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/">GoBilda</a>
     */
    public static final double WHEEL_DIAMETER_MM = 96;

    /**
     * The radius of the mecanum wheels in millimeters
     * @see #WHEEL_DIAMETER_MM
     */
    public static final double WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2;

    /**
     * The amount of encoder ticks in one full revolution
     */
    public static final double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    /**
     * Used convert from motor position (in ticks) to distance (in millimeters)
     */
    public static final double TICKS_TO_MM = ENCODER_RESOLUTION / (WHEEL_RADIUS_MM * 360);

    @ModuleInitializer
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

        final double frontLeftPos = getFrontLeftMecanumDriver().getCurrentPosition();
        final double frontRightPos = getFrontRightMecanumDriver().getCurrentPosition();
        final double backLeftPos = getBackLeftMecanumDriver().getCurrentPosition();
        final double backRightPos = getBackLeftMecanumDriver().getCurrentPosition();

        final double forwardDistance = ((frontLeftPos + frontRightPos + backLeftPos + backRightPos) / 4) * TICKS_TO_MM;
        final double strafeDistance = ((frontLeftPos + frontRightPos - backLeftPos - backRightPos) / 4) * TICKS_TO_MM;
        final double rotation = getIMU().getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT);

        return new LocalizedMovement(forwardDistance, strafeDistance, rotation, this);
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
