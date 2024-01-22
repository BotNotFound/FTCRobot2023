package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;

public class FieldCentricDriveTrain extends DriveTrain {

    /**
     * The IMU
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final IMU getIMU() {
        return hardwareDevices.getLoadedDevice(IMU.class, IMU_NAME);
    }

    private static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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

    private static final double TAU = 2 * Math.PI;

    public void resetRotation() {
        hardwareDevices.executeIfAllAreAvailable(getIMU()::resetYaw);
    }

    private static double findSmallestPositiveCoterminalAngle(double radians) {
        while (radians < 0) {
            radians += TAU; // 1 rotation; the angle is still coterminal
        }

        return radians % TAU;
    }

    @Override
    public void setVelocity(double distX, double distY, double rotation) {
        hardwareDevices.executeIfAllAreAvailable(() -> {
            final double botHeading =
                    findSmallestPositiveCoterminalAngle(
                            getIMU()
                                    .getRobotYawPitchRollAngles()
                                    .getYaw(AngleUnit.RADIANS)
                    );

            // Rotate the movement direction counter to the robot's rotation
            double rotX = distX * Math.cos(-botHeading) - distY * Math.sin(-botHeading);
            double rotY = distX * Math.sin(-botHeading) + distY * Math.cos(-botHeading);
            getTelemetry().addData("[Field Centric Drive Train] bot heading value", botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing

            super.setVelocity(rotX, rotY, rotation);
        });
    }
}
