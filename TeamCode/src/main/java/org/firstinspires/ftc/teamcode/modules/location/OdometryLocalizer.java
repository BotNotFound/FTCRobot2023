package org.firstinspires.ftc.teamcode.modules.location;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A localizer that uses the {@link Odometry} module to get the position
 */
public final class OdometryLocalizer extends Odometry implements Localizer {
    public static final double WHEEL_RADIUS = 0; // TODO

    private Pose2d poseOffset;

    private void clearPoseOffset() {
        poseOffset = new Pose2d(0,0,0);
    }

    public OdometryLocalizer(OpMode registrar) {
        super(registrar);
        clearPoseOffset();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        assert isActive(); // if not active, cannot get location
        LocalizedMovement curPos = getLocation();
        return new Pose2d(curPos.x, curPos.y, curPos.theta).plus(poseOffset);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        clearPoseOffset();
        poseOffset = getPoseEstimate().times(-1).plus(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        AtomicReference<Pose2d> ret = new AtomicReference<>(); // it's atomic because idea gets mad if it isn't

        hardwareDevices.executeIfAllAreAvailable(() -> {
            double frontLeftVelocity = getFrontLeftMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS;
            double frontRightVelocity = getFrontRightMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS;
            double backLeftVelocity = getBackLeftMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS;
            double backRightVelocity = getBackRightMecanumDriver().getVelocity(AngleUnit.RADIANS) * WHEEL_RADIUS;

            final double forwardVelocity = ((frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) / 4) * TICKS_TO_MM;
            final double strafeVelocity = ((frontLeftVelocity + frontRightVelocity - backLeftVelocity - backRightVelocity) / 4) * TICKS_TO_MM;
            final double rotationalVelocity = getIMU().getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate; // I think

            ret.set(new Pose2d(strafeVelocity, forwardVelocity, rotationalVelocity));
        });

        return ret.get();
    }

    @Override
    public void update() {
        // nothing to update
    }
}
