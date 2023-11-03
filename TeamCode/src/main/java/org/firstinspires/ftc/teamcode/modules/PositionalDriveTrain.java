package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Point;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicInteger;

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
     * @see #enqueueDistance(Point)
     * @see #getFirstInQueue() 
     * @see #isInQueue(long) 
     * @see #enqueueAndWait(Point)
     */
    private final Queue<Point> distanceQueue = new LinkedList<>();

    /**
     * The current ID of the last item in {@link #distanceQueue}
     * @see #distanceQueue
     */
    private final AtomicInteger curId = new AtomicInteger();

    /**
     * Adds a distance to the position updater thread queue
     * @param distance the distance to add
     * @return the ID of the distance added to the queue
     * @see #distanceQueue
     * @see #enqueueAndWait(Point)
     */
    private synchronized long enqueueDistance(Point distance) {
        distanceQueue.add(distance);
        return curId.getAndIncrement();
    }

    /**
     * Gets the ID of the first element in the position updater thread queue
     * @return the ID
     * @see #distanceQueue
     * @see #isInQueue(long) 
     */
    private synchronized long getFirstInQueue() {
        return curId.get() - distanceQueue.size();
    }

    /**
     * Checks if the specified distance is still in the queue
     * @param id the id of the distance in the queue
     * @return false if the distance has been traveled, otherwise true
     * @see #getFirstInQueue() 
     */
    private synchronized boolean isInQueue(long id) {
        // the distance most recently removed from the queue is the current input for the
        // position updater thread, so this will only return false when the specified distance
        // has been traveled
        return id + 1 >= getFirstInQueue();
    }

    /**
     * Adds the specified distance to the queue and waits until it has been traveled
     * @param distance the distance to add
     * @throws InterruptedException error in wait()
     * @see #isInQueue(long)
     * @see #enqueueDistance(Point) 
     */
    private void enqueueAndWait(Point distance) throws InterruptedException {
        long id = enqueueDistance(distance);
        while (true) {
            synchronized (distanceQueue) {
                distanceQueue.wait();
                if (!isInQueue(id)) {
                    return;
                }
            }
        }
    }

    /**
     * positionUpdaterThread ends when this is true
     * @see #positionUpdaterThread
     */
    private volatile boolean killUpdaterThread;

    /**
     * The thread that sets the motor speeds based on the remaining distance to travel
     * @see #distanceQueue
     * @see #killUpdaterThread
     * @see #moveAndRotateRobot(double, double, double)
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
                    distanceQueue.notify();
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
            try { Thread.sleep(POSITION_UPDATER_SLEEP_TIME); } catch (InterruptedException e) { // TODO if we don't actually need this, remove it
                synchronized (parent) {
                    parent.telemetry.addData("Error in position updater thread (on Thread.sleep())", e.getMessage());
                }
            } // arbitrary offset
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
     * @see #moveAndRotateRobot(Point)
     */
    public void moveAndRotateRobot(double distX, double distY, double rotation) {
        moveAndRotateRobot(new Point(distX, distY, rotation));
    }

    /**
     * Moves and rotates the robot
     * @param distance The amount of distance to move
     * @see #moveAndRotateRobot(double, double, double) 
     * @see #moveAndWait(Point) 
     */
    public void moveAndRotateRobot(@NonNull Point distance) {
        enqueueDistance(distance);
    }

    /**
     * Waits until the robot has moved the specified distance
     * @param distance the distance that the robot will move
     * @throws InterruptedException the current thread was interrupted
     * @see #moveAndRotateRobot(Point)
     * @see #moveAndWait(double, double, double)
     */
    public void moveAndWait(Point distance) throws InterruptedException {
        enqueueAndWait(distance);
    }

    /**
     * Waits until the robot has moved the specified distance
     * @param distX the distance to move horizontally
     * @param distY the distance to move vertically
     * @param rotation the amount to rotate
     * @throws InterruptedException the current thread was interrupted
     * @see #moveAndRotateRobot(Point)
     */
    public void moveAndWait(double distX, double distY, double rotation) throws InterruptedException {
        moveAndWait(new Point(distX, distY, rotation));
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
