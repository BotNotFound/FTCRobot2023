package org.firstinspires.ftc.teamcode.modules.concurrent;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;

/**
 * Describes the state of a {@link ConcurrentModule}.  Used to allow its threads a chance to detect termination of the
 * OpMode and exit gracefully before they are interrupted.
 */
public enum ModuleRunState {
    /**
     * The module is still within its constructor.  This state is used to prevent registration of new threads after
     * the module has been created.  For all other purposes, it is equivalent to {@link #INIT}.
     */
    NEW,

    /**
     * The module has completed thread initialization, but is still within its constructor.  Child classes should call
     */
    SETUP,

    /**
     * The method {@link ConcurrentModule#startThreads()} has not been called yet.  Typically, this signifies that the
     * parent OpMode is still in init.
     * @apiNote Module threads should NOT use the hardware map to get references to hardware devices.  All hardware
     *  devices should be initialized by the host {@link ConcurrentModule}, and threads are expected to use hardware
     *  devices through it.
     * @see OpMode#init()
     * @see LinearOpMode#opModeInInit()
     */
    INIT,

    /**
     * The parent OpMode is in the main execution loop.  Module threads should only interact with hardware objects when
     * the host module is in this state.
     */
    RUNNING,

    /**
     * The method {@link ModuleBase#cleanupModule()} has been called, and the OpMode has been stopped.  When a module
     * is in this state, threads will have a limited amount of time to clean up before they are forcefully interrupted.
     */
    TERMINATED;

    // the methods below are used by module threads similar to LinearOpMode's opModeInInit() and opModeIsActive() methods

    /**
     * Has the module terminated?
     * @return True if the state is {@link #TERMINATED}, otherwise false
     */
    public boolean isTerminated() {
        return this == TERMINATED;
    }

    /**
     * Is the module still in its constructor?
     * @return True if the state is {@link #NEW} or {@link #SETUP}, otherwise false
     */
    public boolean isInConstructor() {
        return this == NEW || this == SETUP;
    }

    /**
     * Is the module in the initialization phase?
     * @return True if {@link #isInConstructor()} is true or the state is {@link #INIT}, otherwise false
     */
    public boolean isInInit() {
        return isInConstructor() || this == INIT;
    }

    /**
     * Is the module's parent {@link OpMode} in its main execution loop?
     * @return True if the state is {@link #RUNNING}, otherwise false
     */
    public boolean isRunning() {
        return this == RUNNING;
    }
}
