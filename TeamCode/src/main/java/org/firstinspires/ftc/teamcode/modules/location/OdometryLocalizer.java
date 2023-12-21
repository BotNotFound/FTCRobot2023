package org.firstinspires.ftc.teamcode.modules.location;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.annotations.ModuleInitializer;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A localizer that uses the {@link Odometry} module to get the position
 */
public final class OdometryLocalizer extends Odometry implements Localizer {

    /**
     * The angle unit we are using.
     * @implNote Set to radians because that's what roadrunner uses for its headings
     */
    public static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    /**
     * One revolution in {@link #ANGLE_UNIT}
     */
    public static final double ONE_REVOLUTION_OUR_ANGLE_UNIT = ANGLE_UNIT.getUnnormalized().fromDegrees(360);

    /**
     * Gets the field size in the units we're using
     * @return The field size (distance is in inches, rotation is in {@link #ANGLE_UNIT})
     */
    @Override
    public Movement getFieldSize() {
        return new Movement(144, 144, ONE_REVOLUTION_OUR_ANGLE_UNIT);
    }

    /**
     * Gets the current location
     * @return The current location in the units used by this locator
     * @throws LocatorException The locator was unable to retrieve the location.  This is most likely due to the drive
     *  train or IMU not being plugged in or configured correctly
     * @see #getFieldSize()
     */
    @Override
    public synchronized LocalizedMovement getLocation() throws LocatorException {
        // Normally, we would just use the convertToOtherLocator method.  However, that method calls getLocation() to
        //  calculate the offset for conversion, so calling it here would result in an infinite loop.  Luckily, both this
        //  class and its parent have the same starting position, so we can leave out the offsets

        final LocalizedMovement superLocation = super.getLocation();
        final Movement superFieldSize = super.getFieldSize();
        final Movement thisFieldSize = this.getFieldSize();
        return new LocalizedMovement(
            superLocation.x * thisFieldSize.x / superFieldSize.x,
            superLocation.y * thisFieldSize.y / superFieldSize.y,
            superLocation.theta * thisFieldSize.theta / superFieldSize.theta,
            this
        );
    }

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

    /**
     * Initializes this module
     * @param registrar The OpMode that is initializing this
     */
    @ModuleInitializer
    public OdometryLocalizer(OpMode registrar) {
        super(registrar);
        offset = Movement.zero();
    }

    /**
     * Gets the current position of the robot
     * @return The robot's current position
     * @throws LocatorException The call to {@link #getLocation()} failed
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        LocalizedMovement curPos = getLocation();
        return curPos.toPose().plus(getOffset());
    }

    /**
     * Overridden from {@link Localizer}.  Gets the current position of the robot
     * @param pose2d The robot's position (x is strafe distance, y is forward distance, and heading is rotation)
     * @throws LocatorException The call to {@link #getLocation()} failed
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
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
            final double rotationalVelocity = getIMU().getRobotAngularVelocity(ANGLE_UNIT).zRotationRate;

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
