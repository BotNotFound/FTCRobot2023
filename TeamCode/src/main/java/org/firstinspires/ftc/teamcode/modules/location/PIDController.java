package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.modules.FieldCentricDriveTrain;

/**
 * A drive-to-position variant of {@link DriveTrain} that uses a
 *  Proportional, Derivative, and Integral system to control the robot
 */
public class PIDController extends FieldCentricDriveTrain { // TODO TUNE THE PID CONTROLLER
    /**
     * The proportional coefficient
     */
    public static final double KP = 1;

    /**
     * The derivative coefficient
     */
    public static final double KD = 0;

    /**
     * The integral coefficient
     */
    public static final double KI = 0;

    /**
     * The maximum power the drive train can provide.
     * This value is used for Integrator clamping
     */
    public static final double INTEGRAL_SUM_LIMIT = 1;

    /**
     * Used in the low-pass filter for the derivative term. <br />
     * <b>MUST BE BETWEEN 0 AND 1 (exclusive)</b>
     */
    public static final double A = 0.8;


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
        public double filterEstimate;

        public MovementInfo() {
            error = 0;
            derivative = 0;
            integralSum = 0;
            filterEstimate = 0;
        }
    }

    /**
     * Moves the robot to the target position
     * @param target The target position
     */
    public void driveTo(LocalizedMovement target) {
        if (target == null) { return; }

        Locator locator = target.getLocator();
        if (locator.getKind() == LocatorKind.NO_ABSOLUTE_POSITION) {
            throw new NoAbsolutePositionException(locator);
        }

        MovementInfo infoX = new MovementInfo();
        MovementInfo infoY = new MovementInfo();
        MovementInfo infoRotation = new MovementInfo();
        Movement velocity = Movement.zero();
        LocalizedMovement currentPosition;
        ElapsedTime timer = new ElapsedTime();
        double deltaTime;

        do {
            if (!locator.isActive()) {
                throw new RuntimeException("Provided locator was inactive!");
            }

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
            velocity.theta = calcVelocity(currentPosition.theta,
                    target.theta,
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

        // filter out height frequency noise to increase derivative performance
        double errorChange = (info.error - lastError);
        info.filterEstimate = (A * info.filterEstimate) + (1-A) * errorChange;

        // rate of change of the error
        info.derivative = info.filterEstimate / deltaTime;

        // sum of all error over time
        info.integralSum += info.error * deltaTime;

        // set a limit on our integral sum
        if (info.integralSum > INTEGRAL_SUM_LIMIT) {
            info.integralSum = INTEGRAL_SUM_LIMIT;
        }
        else if (info.integralSum < -INTEGRAL_SUM_LIMIT) {
            info.integralSum = -INTEGRAL_SUM_LIMIT;
        }

        return (KP * info.error) + (KI * info.integralSum) + (KD * info.derivative);
    }
}
