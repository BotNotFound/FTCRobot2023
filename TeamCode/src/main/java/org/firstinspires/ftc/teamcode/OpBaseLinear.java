package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class OpBaseLinear extends OpBase {

    /**
     * Used to ensure we only call {@link #requestOpModeStop()} once
     * @see #loop()
     */
    private final AtomicBoolean stopRequested;

    public OpBaseLinear() {
        super();
        stopRequested = new AtomicBoolean(false);
    }

    /**
     * Unused
     * @see #loop()
     */
    @Override
    public final void init_loop() {
        super.init_loop();
    }

    /**
     * OpMode's start().  Calls {@link #runOpMode()}
     * @see #runOpMode()
     * @see OpMode#start()
     */
    @Override
    public final void start() {
        super.start();
        runOpMode();
    }

    /**
     * OpMode's loop() method
     * @see OpMode#loop()
     */
    @Override
    public final void loop() {
        if (stopRequested.compareAndSet(false, true)) {
            requestOpModeStop();
        }
        Thread.yield(); // everything in this OpMode is done at start()
    }

    /**
     * Contains the actual code for the OpMode
     * @see LinearOpMode#runOpMode()
     */
    public abstract void runOpMode();
}
