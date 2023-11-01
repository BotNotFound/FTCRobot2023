package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Point;

import java.util.LinkedList;
import java.util.Queue;

/**
 * A drive train that can drive to positions
 */
public final class PositionalDriveTrain extends DriveTrain {
    /**
     * The delay between each cycle of the position updater thread loop
     */
    public static final long POSITION_UPDATER_SLEEP_TIME = 10;

    /**
     * The requested changes in position, in order of time requested
     */
    private final Queue<Point> distanceQueue = new LinkedList<>();

    /**
     * positionUpdaterThread ends when this is true
     */
    private boolean killUpdaterThread;

    /**
     * The thread that sets the motor speeds based on the remaining distance to travel
     */
    private final Thread positionUpdaterThread = new Thread(() -> {
        double prevTime = parent.getRuntime(),
                deltaTime, curRuntime;
        Point threadSafeRemainingDistance = new Point(0,0,0),
                distanceOffset;

        while (!killUpdaterThread) {
            curRuntime = parent.getRuntime();
            deltaTime = curRuntime - prevTime;
            prevTime = curRuntime;

            if (threadSafeRemainingDistance.x == 0 &&
                    threadSafeRemainingDistance.y == 0 &&
                    threadSafeRemainingDistance.rotation == 0) {
                synchronized (distanceQueue) {
                    threadSafeRemainingDistance = distanceQueue.remove();
                }
            }

            // rotate, then move
            if (threadSafeRemainingDistance.rotation != 0) {
                double absRemainingRotation = Math.abs(threadSafeRemainingDistance.rotation);
                // (same w/ x & y, distance is measured in nanoseconds) if deltaTime > remaining
                //  distance, set motors to fractional power; otherwise, just set to full power
                if (threadSafeRemainingDistance.rotation < deltaTime) {
                    setVelocity(0, 0, Math.copySign(absRemainingRotation / deltaTime, threadSafeRemainingDistance.rotation));
                    distanceOffset = new Point(0,0,-threadSafeRemainingDistance.rotation);
                }
                else {
                    setVelocity(0, 0, Math.copySign(1, threadSafeRemainingDistance.rotation));
                    distanceOffset = new Point(0,0,-Math.copySign(deltaTime, threadSafeRemainingDistance.rotation));
                }
            }
            else if (threadSafeRemainingDistance.x != 0 || threadSafeRemainingDistance.y != 0) {
                double absRemainingDistX = Math.abs(threadSafeRemainingDistance.x);
                double absRemainingDistY = Math.abs(threadSafeRemainingDistance.y);
                double powerX = Math.copySign(absRemainingDistX < deltaTime ? absRemainingDistX / deltaTime : 1, threadSafeRemainingDistance.x);
                double powerY = Math.copySign(absRemainingDistY < deltaTime ? absRemainingDistY / deltaTime : 1, threadSafeRemainingDistance.y);
                setVelocity(powerX, powerY, 0);
                distanceOffset = new Point(Math.copySign(Math.max(absRemainingDistX, deltaTime), threadSafeRemainingDistance.x),
                        Math.copySign(Math.max(absRemainingDistY, deltaTime), threadSafeRemainingDistance.y),
                        0).negate();
            }
            else {
                setVelocity(0, 0, 0);
                distanceOffset = new Point(0,0,0);
            }

            threadSafeRemainingDistance = threadSafeRemainingDistance.add(distanceOffset);
            try { Thread.sleep(POSITION_UPDATER_SLEEP_TIME); } catch (InterruptedException e) {} // arbitrary offset
        }
    }, "Position Updater Thread");

    /**
     * Initializes the drive train
     * @param registrar The parent OpMode
     * @throws InterruptedException Initialization failed
     */
    public PositionalDriveTrain(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);

        killUpdaterThread = false;
        positionUpdaterThread.start();
    }


    /**
     * Moves and rotates the robot
     * @param distX the distance to move horizontally
     * @param distY the distance to move vertically
     * @param rotation the amount to rotate
     */
    public void moveAndRotateRobot(double distX, double distY, double rotation) {
        moveAndRotateRobot(new Point(distX, distY, rotation));
    }

    /**
     * Moves and rotates the robot
     * @param distance The amount of distance to move
     */
    public void moveAndRotateRobot(@NonNull Point distance) {
        synchronized (distanceQueue) {
            distanceQueue.add(distance);
        }
    }

    /**
     * Cleans up the module and stops the updater thread
     */
    @Override
    public void cleanupModule() {
        super.cleanupModule();
        killUpdaterThread = true;
        try {
            positionUpdaterThread.join();
        } catch (InterruptedException e) {
            getTelemetry().addData("ERROR at position updater thread", e.getMessage());
        }
    }
}
