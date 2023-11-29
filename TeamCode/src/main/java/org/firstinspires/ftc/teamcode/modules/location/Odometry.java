package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

import java.util.concurrent.TimeUnit;

public class Odometry extends FieldCentricDriveTrain implements Locator {
    /**
     * The radius of the mecanum wheels in millimeters
     */
    public static final double WHEEL_RADIUS = 48;

    /**
     * The amount of encoder ticks in one full revolution
     */
    public static final double ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    public static final double TICKS_TO_MM = ENCODER_RESOLUTION / (WHEEL_RADIUS * 2 * Math.PI);

    private final ElapsedTime timer;

    private Movement currentPosition;

    public static final double PRECISION = 0.001;

    public static final double MIN_UPDATE_INTERVAL = 1;

    public Odometry(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);

        timer = new ElapsedTime();
        currentPosition = Movement.zero();
    }

    synchronized void updateOdometry() {
        final double deltaTime = timer.time();
        if (deltaTime < MIN_UPDATE_INTERVAL) {
            return; // to avoid floating point shenanigans
        }
        timer.reset();

        final double angle = imu.getRobotYawPitchRollAngles().getYaw(ANGLE_UNIT);

        // apply mecnaum kinematic model (with wheel velocities [ticks per sec])
        double xV = (frontLeftMecanumDriver.getVelocity() + frontRightMecanumDriver.getVelocity()
                + backLeftMecanumDriver.getVelocity() + backRightMecanumDriver.getVelocity()) * 0.5;

        double yV =  (-frontLeftMecanumDriver.getVelocity() + frontRightMecanumDriver.getVelocity()
                + backLeftMecanumDriver.getVelocity() - backRightMecanumDriver.getVelocity()) * 0.5;

        // rotate the vector
        double nx = (xV*Math.cos(Math.toRadians(angle)))-(yV*Math.sin(Math.toRadians(angle)));
        double nY = (xV*Math.sin(Math.toRadians(angle)))+(yV*Math.cos(Math.toRadians(angle)));
        xV = nx; yV = nY;

        // integrate velocity over time
        currentPosition.x += (yV*deltaTime)/TICKS_TO_MM; // <-- Tick to inch conversion factor
        currentPosition.y += (xV*deltaTime)/TICKS_TO_MM;
        currentPosition.theta = angle;

        // round floats
        currentPosition.x = Math.round(currentPosition.x / PRECISION) * PRECISION;
        currentPosition.y = Math.round(currentPosition.y / PRECISION) * PRECISION;
        currentPosition.theta = Math.round(currentPosition.theta / PRECISION) * PRECISION;
    }

    protected synchronized void resetTimer() {
        timer.reset();
    }

    @Override
    public synchronized LocalizedMovement getLocation() throws LocatorException {
        return LocalizedMovement.construct(currentPosition, this);
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
        return new Movement(144, 144, 2 * Math.PI);
    }
}
