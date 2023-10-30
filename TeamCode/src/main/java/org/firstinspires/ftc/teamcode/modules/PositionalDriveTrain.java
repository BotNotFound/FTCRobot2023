package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Point;

public final class PositionalDriveTrain extends DriveTrain {
    private Point remainingDistance;

    private boolean killUpdaterThread;
    private final Thread positionUpdaterThread;

    public PositionalDriveTrain(@NonNull OpMode registrar) throws InterruptedException {
        super(registrar);

        positionUpdaterThread = new Thread(() -> {
            double prevTime = parent.getRuntime();
            double deltaTime;
            Point threadSafeRemainingDistance, distanceOffset;

            while (!killUpdaterThread) {
                deltaTime = parent.getRuntime() - prevTime;
                prevTime = parent.getRuntime();

                synchronized (remainingDistance) {
                    threadSafeRemainingDistance = remainingDistance;
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

                synchronized (remainingDistance) {
                    remainingDistance = remainingDistance.add(distanceOffset);
                }
                try { Thread.sleep(10); } catch (InterruptedException e) {} // arbitrary offset
            }
        });

        remainingDistance = new Point(0, 0, 0);
        killUpdaterThread = false;
        positionUpdaterThread.start();
    }



    
    public void moveAndRotateRobot(double distX, double distY, double rotation) {
        synchronized (remainingDistance) {
            remainingDistance = remainingDistance.add(new Point(distX, distY, rotation));
        }
    }

    public void moveAndRotateRobot(@NonNull Point distance) {
        moveAndRotateRobot(distance.x, distance.y, distance.rotation);
    }

    @Override
    public void cleanupModule() {
        super.cleanupModule();
        killUpdaterThread = true;
        positionUpdaterThread.join();
    }
}
