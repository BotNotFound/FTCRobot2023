package org.firstinspires.ftc.teamcode.modules.location;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A localizer that uses the {@link Odometry} module to get the position
 */
public final class OdometryLocalizer extends Odometry implements Localizer {

    /**
     * Set by {@link #setPoseEstimate(Pose2d)} so that {@link #getPoseEstimate()} will actually return an updated position
     * @see #getOffset()
     */
    private Movement offset;

    /**
     * Gets the current positional offset
     * @return The difference between the value returned by {@link #getPoseEstimate()} and the robot's location relative
     *  to its starting position
     * @see #getPoseEstimate()
     * @see Odometry#getLocation()
     */
    public Pose2d getOffset() {
        return offset.toPose();
    }

    public OdometryLocalizer(OpMode registrar) {
        super(registrar);
        offset = Movement.zero();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        assert isActive(); // if not active, cannot get location
        LocalizedMovement curPos = getLocation();
        return curPos.toPose().plus(getOffset());
    }

    /**
     * Overridden from {@link Localizer}.  Gets the current position of the robot
     * @param pose2d The robot's position (x is strafe distance, y is forward distance, and heading is rotation)
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        assert isActive(); // if not active, cannot calculate offset
        offset = getLocation().subtract(Movement.fromPose(pose2d));
    }

    /**
     * Gets the current velocity of the robot
     * @return The velocity (in mm/sec for x & y, and in deg/sec for rotation)
     * @see com.qualcomm.robotcore.hardware.IMU
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        AtomicReference<Pose2d> ret = new AtomicReference<>(); // it's atomic because idea gets mad if it isn't

        hardwareDevices.executeIfAllAreAvailable(() -> {
            // Unit Conversion: [ getVelocity() ]--> rad/sec --[ * radius (mm) ]--> mm/sed
            double frontLeftVelocity = getFrontLeftMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS_MM;
            double frontRightVelocity = getFrontRightMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS_MM;
            double backLeftVelocity = getBackLeftMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS_MM;
            double backRightVelocity = getBackRightMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS_MM;

            final double forwardVelocity = ((frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) / 4);
            final double strafeVelocity = ((frontLeftVelocity + frontRightVelocity - backLeftVelocity - backRightVelocity) / 4);
            // The IMU treats positive Z rotation as clockwise btw
            final double rotationalVelocity = getIMU().getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            ret.set(new Pose2d(strafeVelocity, forwardVelocity, rotationalVelocity));
        });

        return ret.get();
    }

    /**
     * Overridden from {@link Localizer}.  Does nothing because {@link #getPoseEstimate()} and {@link #getPoseVelocity()}
     *  get their values directly from the motors.
     */
    @Override
    public void update() {
        // nothing to update
    }
}
