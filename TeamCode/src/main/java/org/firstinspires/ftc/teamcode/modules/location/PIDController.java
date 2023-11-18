package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;

/**
 * A drive-to-position variant of {@link DriveTrain} that uses a
 *  Proportional, Derivative, and Integral system to control the robot
 */
public class PIDController extends DriveTrain { // TODO TUNE THE PID CONTROLLER
    /**
     * The proportional coefficient
     */
    public static double KP = 1;

    /**
     * The derivative coefficient
     */
    public static double KD = 0;

    /**
     * The integral coefficient
     */
    public static double KI = 0;

    /**
     * Attempts to initialize the module by getting motors with the default names from a hardware map
     *
     * @param registrar the OpMode that will be using the module
     * @throws InterruptedException The module was unable to locate the necessary motors
     */
    public PIDController(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);
    }

    /**
     * Internal class for PID data
     */
    private static final class MovementInfo {
        public double error;
        public double derivative;
        public double integralSum;

        public MovementInfo() {
            error = 0;
            derivative = 0;
            integralSum = 0;
        }
    }

    /**
     * Moves the robot to the target position
     * @param target The target position
     */
    public void driveTo(LocalizedMovement target) {
        if (target == null) { return; }

        Locator locator = target.getLocator();
        MovementInfo infoX = new MovementInfo();
        MovementInfo infoY = new MovementInfo();
        MovementInfo infoRotation = new MovementInfo();
        Movement velocity = Movement.zero();
        LocalizedMovement currentPosition;
        ElapsedTime timer = new ElapsedTime();
        double deltaTime;

        do {
            deltaTime = timer.seconds();
            currentPosition = locator.getLocation();
            velocity.x = calcVelocity(currentPosition.x,
                    target.x,
                    infoX,
                    deltaTime);
            velocity.y = calcVelocity(currentPosition.y,
                    target.y,
                    infoY,
                    deltaTime);
            velocity.rotation = calcVelocity(currentPosition.rotation,
                    target.rotation,
                    infoRotation,
                    deltaTime);
            setVelocity(velocity);
            timer.reset();
        } while (!currentPosition.equals(target));
    }

    private double calcVelocity(
            double currentPosition,
            double targetPosition,
            MovementInfo info,
            double deltaTime) {
        double lastError = info.error;
        info.error = targetPosition - currentPosition;
        info.derivative = (info.error - lastError) / deltaTime;
        info.integralSum += info.error * deltaTime;
        return (KP * info.error) + (KI * info.integralSum) + (KD * info.derivative);
    }
}
