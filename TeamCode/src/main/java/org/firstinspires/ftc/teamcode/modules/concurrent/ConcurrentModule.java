package org.firstinspires.ftc.teamcode.modules.concurrent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;

/**
 * Represents a module that utilizes external threads
 */
public abstract class ConcurrentModule extends ModuleBase {

    /**
     * The current state of the module
     * @see ModuleRunState
     * @implNote This is volatile because it's only set here, but all module threads should be checking this for their
     *  loop condition
     */
    private volatile ModuleRunState state;

    /**
     * The time, in milliseconds, that the module will wait after {@link #cleanupModule()} is called before interrupting its
     * threads.
     * @implNote The last time this was updated, the robot's internal robot stuck detector would wait 10000 ms before
     *  force quitting the OpMode.  This should be less than that
     */
    public static final double MODULE_THREAD_TERMINATION_TIMEOUT_MILLIS = 500;

    /**
     * Gets the current state of the module
     * @return The module's state
     */
    public ModuleRunState getState() {
        return state;
    }

    /**
     * The thread group all the module's threads are a part of
     * @implNote This is package-private because it is used in {@link ModuleThread}'s constructors.  It shouldn't be
     *  visible to child classes.
     */
    /* package-private */ final ThreadGroup moduleThreadGroup;

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     * @param threadGroupName The name of the thread group that will contain all of this module's threads
     */
    public ConcurrentModule(OpMode registrar, String threadGroupName) {
        super(registrar);
        moduleThreadGroup = new ThreadGroup(threadGroupName);
        state = ModuleRunState.NEW;
        registerModuleThreads();
        state = ModuleRunState.INIT;
    }

    /**
     * Overridden by child classes to register all necessary threads
     * @see #registerAsyncOperation(ModuleThread)
     */
    protected abstract void registerModuleThreads();

    /**
     * Registers a module thread, which will run asynchronously until the module cleans up (the parent OpMode terminates)
     * @param thread The thread to register
     * @throws IllegalStateException Attempted to register a thread outside of {@link #registerModuleThreads()}
     * @throws IllegalArgumentException The provided {@link ModuleThread} belongs to a different host module
     * @apiNote This should only be called in {@link #registerModuleThreads()}
     */
    protected final void registerAsyncOperation(ModuleThread<?> thread) {
        if (state != ModuleRunState.NEW) {
            throw new IllegalStateException("Cannot register module threads outside of constructor!");
        }

        if (thread.host == this) {
            throw new IllegalArgumentException("Module threads may only be registered by their host module!");
        }
    }

    /**
     * Signals to this module's threads that the parent {@link OpMode} has entered its main execution loop
     */
    public final void startThreads() {
        state = ModuleRunState.RUNNING;
    }

    /**
     * Signals to the module's threads that the parent {@link OpMode} has terminated and interrupts them after the duration
     * specified by {@link #MODULE_THREAD_TERMINATION_TIMEOUT_MILLIS}
     */
    private void endThreads() {
        state = ModuleRunState.TERMINATED;
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < MODULE_THREAD_TERMINATION_TIMEOUT_MILLIS) {
            if (moduleThreadGroup.activeCount() == 0) {
                moduleThreadGroup.destroy();
                return; // we don't have to wait the entire time if all threads end gracefully
            }
            Thread.yield(); // give the threads a chance to exit gracefully
        }

        // We have to kill all threads before the robot thinks we're stuck and dies painfully
        moduleThreadGroup.interrupt();
        moduleThreadGroup.destroy();
    }

    /**
     * Interrupts this module's threads
     */
    public void interrupt() {
        moduleThreadGroup.interrupt();
    }

    /**
     * Overridden in child classes to clean up the module's state when the parent {@link OpMode}.  Terminates all threads.
     */
    @Override
    public void cleanupModule() {
        endThreads();
    }
}
