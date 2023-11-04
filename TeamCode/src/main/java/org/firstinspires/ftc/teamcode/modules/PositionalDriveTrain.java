package org.firstinspires.ftc.teamcode.modules;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Point;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * A drive train that can drive to positions
 */
public final class PositionalDriveTrain extends DriveTrain {

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
        return id >= getFirstInQueue();
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
    private AtomicBoolean killUpdaterThread;

    /**
     * Gets the desired velocity and distance traveled from the specified distance
     * @param remainingDistance the remaining distance to travel on this axis
     * @param deltaTime the time since the last update
     * @return the distance traveled (first) and the power to supply to the motor (second)
     */
    private Pair<Double, Double> getVelocityFromDistance(double remainingDistance, double deltaTime) {
        double absRemainingDistance = Math.abs(remainingDistance);
        // (same w/ x & y, distance is measured in nanoseconds) if deltaTime > remaining
        //  distance, set motors to fractional power; otherwise, just set to full power
        if (absRemainingDistance < deltaTime) {
            return new Pair<>(-remainingDistance, Math.copySign(absRemainingDistance / deltaTime, remainingDistance));
        } else {
            return new Pair<>(-Math.copySign(deltaTime, remainingDistance), Math.copySign(1, remainingDistance));
        }
    }

    /**
     * The thread that sets the motor speeds based on the remaining distance to travel
     * @see #distanceQueue
     * @see #killUpdaterThread
     * @see #moveAndRotateRobot(double, double, double)
     */
    private final Thread positionUpdaterThread = new Thread(() -> {
        try {
            double prevTime = parent.getRuntime(),
                    deltaTime, curRuntime;
            Point threadSafeRemainingDistance = new Point(0, 0, 0),
                    distanceOffset;
            boolean consumeFirstInQueue = false;

            while (!killUpdaterThread.get()) {
                curRuntime = parent.getRuntime();
                deltaTime = curRuntime - prevTime;
                prevTime = curRuntime;

                if (threadSafeRemainingDistance.x == 0 &&
                        threadSafeRemainingDistance.y == 0 &&
                        threadSafeRemainingDistance.rotation == 0) {
                    synchronized (distanceQueue) {
                        if (distanceQueue.size() == 0) { // nothing to do
                            Thread.yield();
                            continue;
                        }

                        if (consumeFirstInQueue) {
                            distanceQueue.remove();
                            distanceQueue.notify();
                        }
                        threadSafeRemainingDistance = distanceQueue.element();
                    }
                }

                // rotate, then move
                if (threadSafeRemainingDistance.rotation != 0) { getTelemetry().addLine("rotating...");
                    Pair<Double, Double> rotAndVelo = getVelocityFromDistance(threadSafeRemainingDistance.rotation, deltaTime);
                    setVelocity(0,0,rotAndVelo.second);
                    distanceOffset = Point.Axis.ROTATION.genPointFromAxis(rotAndVelo.first);
                } else if (threadSafeRemainingDistance.x != 0 || threadSafeRemainingDistance.y != 0) { getTelemetry().addLine("moving...");
                    Pair<Double, Double> distXAndVelo = getVelocityFromDistance(threadSafeRemainingDistance.x, deltaTime);
                    Pair<Double, Double> distYAndVelo = getVelocityFromDistance(threadSafeRemainingDistance.y, deltaTime);
                    setVelocity(distXAndVelo.second, distYAndVelo.second, 0);
                    distanceOffset = Point.Axis.X.genPointFromAxis(distXAndVelo.first).add(Point.Axis.Y.genPointFromAxis(distYAndVelo.first));
                } else { getTelemetry().addLine("idle");
                    setVelocity(0, 0, 0);
                    threadSafeRemainingDistance = new Point(0,0,0);
                    distanceOffset = new Point(0, 0, 0);
                    consumeFirstInQueue = true;
                }

                threadSafeRemainingDistance = threadSafeRemainingDistance.add(distanceOffset);
                //noinspection BusyWait
                Thread.sleep(1, 0);
            }
        }
        catch (Throwable th) { // NO MORE SILENT FAILURES
            synchronized (parent) {
                parent.telemetry.addData("ERROR in Position Updater Thread", th.getMessage());
                for (StackTraceElement element : th.getStackTrace()) {
                    parent.telemetry.addData("] at", element.toString());
                }
            }
        }
    }, "Position Updater Thread");

    /**
     * Initializes the drive train
     * @param registrar The parent OpMode
     * @throws InterruptedException Initialization failed
     */
    public PositionalDriveTrain(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);

        killUpdaterThread = new AtomicBoolean(false);
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
        killUpdaterThread.lazySet(true);
        try {
            positionUpdaterThread.join();
        } catch (InterruptedException e) {
            getTelemetry().addData("ERROR at position updater thread", e.getMessage());
        }
    }
}
