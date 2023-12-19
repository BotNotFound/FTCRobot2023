package org.firstinspires.ftc.teamcode.modules;

import android.util.Pair;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDeviceGroup;

public class DriveTrain extends ModuleBase {
    /**
     * The motor that drives the front right mecanum wheel
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final DcMotorEx getFrontRightMecanumDriver() {
        return hardwareDevices.getLoadedDevice(DcMotorEx.class, FRONT_RIGHT_MECANUM_DRIVER_DEFAULT_NAME);
    }

    /**
     * The default name of the front right mecanum driver
     */
    public static final String FRONT_RIGHT_MECANUM_DRIVER_DEFAULT_NAME = "Front Right Mecanum Driver";


    /**
     * The motor that drives the front left mecanum wheel
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final DcMotorEx getFrontLeftMecanumDriver() {
        return hardwareDevices.getLoadedDevice(DcMotorEx.class, FRONT_LEFT_MECANUM_DRIVER_DEFAULT_NAME);
    }


    /**
     * The default name of the front left mecanum driver
     */
    public static final String FRONT_LEFT_MECANUM_DRIVER_DEFAULT_NAME = "Front Left Mecanum Driver";


    /**
     * The motor that drives the back right mecanum wheel
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final DcMotorEx getBackRightMecanumDriver() {
        return hardwareDevices.getLoadedDevice(DcMotorEx.class, BACK_RIGHT_MECANUM_DRIVER_DEFAULT_NAME);
    }


    /**
     * The default name of the back right mecanum driver
     */
    public static final String BACK_RIGHT_MECANUM_DRIVER_DEFAULT_NAME = "Back Right Mecanum Driver";

    /**
     * The motor that drives the back left mecanum wheel
     * @apiNote This should only be called within the
     * {@link ConditionalHardwareDeviceGroup#executeIfAllAreAvailable(Runnable)} of {@link #hardwareDevices}
     */
    protected final DcMotorEx getBackLeftMecanumDriver() {
        return hardwareDevices.getLoadedDevice(DcMotorEx.class, BACK_LEFT_MECANUM_DRIVER_DEFAULT_NAME);
    }


    /**
     * The default name of the back left mecanum driver
     */
    public static final String BACK_LEFT_MECANUM_DRIVER_DEFAULT_NAME = "Back Left Mecanum Driver";

    /**
     * A {@link ConditionalHardwareDeviceGroup} containing all the hardware devices necessary for the drive train to function
     */
    protected final ConditionalHardwareDeviceGroup hardwareDevices;

    /**
     * Attempts to initialize the module by getting motors with the default names from a hardware map
     * @param registrar the OpMode that will be using the module
     */
    public DriveTrain(@NonNull OpMode registrar) {
        super(registrar);

        hardwareDevices = new ConditionalHardwareDeviceGroup();
        hardwareDevices.tryLoadDevices(parent.hardwareMap,
                new Pair<>(DcMotorEx.class, FRONT_RIGHT_MECANUM_DRIVER_DEFAULT_NAME),
                new Pair<>(DcMotorEx.class, FRONT_LEFT_MECANUM_DRIVER_DEFAULT_NAME),
                new Pair<>(DcMotorEx.class, BACK_RIGHT_MECANUM_DRIVER_DEFAULT_NAME),
                new Pair<>(DcMotorEx.class, BACK_LEFT_MECANUM_DRIVER_DEFAULT_NAME)
                );

        hardwareDevices.executeIfAllAreAvailable(() -> {

            // motor config
            getFrontRightMecanumDriver().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getBackRightMecanumDriver().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getFrontLeftMecanumDriver().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            getBackLeftMecanumDriver().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            configureMotorDirections(
                    getFrontLeftMecanumDriver(),
                    getFrontRightMecanumDriver(),
                    getBackLeftMecanumDriver(),
                    getBackRightMecanumDriver()
            );

            getTelemetry().addLine("[Drive Train] Found all drive motors");
        }, () -> getTelemetry().addLine("[Drive Train] Could not find all drive motors!"));
    }

    /**
     * Sets the directions of the drive train motors.  Used in {@link #DriveTrain(OpMode)} and
     *  {@link org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive#SampleMecanumDrive(OpMode)}
     * @param frontLeft The front left motor
     * @param frontRight The front right motor
     * @param backLeft The back left motor
     * @param backRight The back right motor
     */
    public static void configureMotorDirections(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        // ensure that we are actually configuring the drive train motors
        assert frontLeft.getDeviceName().equals(FRONT_LEFT_MECANUM_DRIVER_DEFAULT_NAME);
        assert frontRight.getDeviceName().equals(FRONT_RIGHT_MECANUM_DRIVER_DEFAULT_NAME);
        assert backLeft.getDeviceName().equals(BACK_LEFT_MECANUM_DRIVER_DEFAULT_NAME);
        assert backRight.getDeviceName().equals(BACK_RIGHT_MECANUM_DRIVER_DEFAULT_NAME);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void cleanupModule() {
        // nothing to clean up
    }

    @Override
    public void log() {
        // nothing to log
    }

    /**
     * the scale for our exponential scaling of motor power
     */
    public static final int POWER_SCALE = 5;

    /**
     * the scale for our linear scaling of motor power
     */
    public static final double SCALE = 0.75;

    /**
     * Moves and rotates the robot
     * @param distX The right velocity
     * @param distY The forward velocity
     * @param rotation The rotational velocity
     */
    public void setVelocity(double distX, double distY, double rotation) {
        hardwareDevices.executeIfAllAreAvailable(() -> {
            getTelemetry().addData("[Drive Train] Moving by vector:", "<%f, %f, %f>", distX, distY, rotation);

            // Combine the requests for each axis-motion to determine each wheel's power.
            // (formula was found on gm0)
            double leftFrontPower = distY + distX + rotation;
            double leftBackPower = distY - distX + rotation;
            double rightFrontPower = distY - distX - rotation;
            double rightBackPower = distY + distX - rotation;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontPower = Math.pow(leftFrontPower, POWER_SCALE) * SCALE;
            rightFrontPower = Math.pow(rightFrontPower, POWER_SCALE) * SCALE;
            rightBackPower = Math.pow(rightBackPower, POWER_SCALE) * SCALE;
            leftBackPower = Math.pow(leftBackPower, POWER_SCALE) * SCALE;

            getTelemetry().addData("Setting motor power", "%f, %f, %f, %f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Send calculated power to wheels
            getFrontLeftMecanumDriver().setPower(leftFrontPower);
            getFrontRightMecanumDriver().setPower(rightFrontPower);
            getBackRightMecanumDriver().setPower(rightBackPower);
            getBackLeftMecanumDriver().setPower(leftBackPower);
        });
    }

    /**
     * Moves and rotates the robot
     * @param velocity The desired robot
     */
    public void setVelocity(Movement velocity) {
        setVelocity(velocity.x, velocity.y, velocity.theta);
    }
}
