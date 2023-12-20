package org.firstinspires.ftc.teamcode.modules.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.modules.concurrent.ConcurrentModule;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.LinkedList;
import java.util.List;

/**
 * A class that manages the life cycle of modules used by {@link OpMode}s
 */
public final class ModuleManager {
    /**
     * Has the OpMode already signaled for threads in {@link ConcurrentModule}s to start?
     */
    private boolean areThreadsStarted;

    /**
     * The {@link OpMode} that created this
     */
    private final OpMode opMode;

    /**
     * A collection of the loaded modules
     */
    private final List<Module> loadedModules;

    /**
     * Initializes the module manager
     * @param registrar The {@link OpMode} initializing this class
     */
    public ModuleManager(OpMode registrar) {
        opMode = registrar;
        loadedModules = new LinkedList<>();
        areThreadsStarted = false;
    }

    /**
     * Retrieves an instance of a module, initializing it if no instance is available
     * @param moduleClass The class of the module to get
     * @return The module
     * @param <T> The type of the module to get
     * @throws IllegalArgumentException The provided class is abstract
     * @throws ExceptionInInitializerError The module's constructor threw an exception
     * @throws NoSuchMethodError The module's class definition does not have a public constructor taking an {@link OpMode}
     *  as its only parameter
     * @see Module#Module(OpMode)  Module
     */
    public <T extends Module> T getModule(Class<T> moduleClass) throws NoSuchMethodError, IllegalArgumentException, ExceptionInInitializerError {
        if (Modifier.isAbstract(moduleClass.getModifiers())) {
            throw new IllegalArgumentException("Trying to retrieve an instance an abstract class!");
        }

        for (Module module : loadedModules) {
            if (moduleClass.isInstance(module)) {
                return moduleClass.cast(module);
            }
        }

        // no module of the specified type exists
        try {
            Constructor<T> moduleConstructor = moduleClass.getConstructor(OpMode.class);
            T moduleInstance = moduleConstructor.newInstance(opMode);

            if (moduleInstance instanceof ConcurrentModule && areThreadsStarted) {
                ((ConcurrentModule)moduleInstance).startThreads();
            }

            loadedModules.add(moduleInstance);
            return moduleInstance;
        }
        catch (NoSuchMethodException | IllegalAccessException e) {
            // if constructor is private/nonexistent, we can't create an instance
            throw new NoSuchMethodError("Unable to retrieve module constructor with the required signature");
        }
        catch (InstantiationException shouldNeverBeThrown) {
            // If the provided class is abstract, fail.
            //  We check if the class is abstract at the beginning of the method and fail there, so this code should
            //  never be reached.
            throw new IllegalArgumentException(shouldNeverBeThrown);
        }
        catch (InvocationTargetException passOn) {
            // If the constructor fails, pass on the error
            throw new ExceptionInInitializerError(passOn.getTargetException());
        }
    }

    /**
     * Starts all {@link org.firstinspires.ftc.teamcode.modules.concurrent.ModuleThread}s belonging to any loaded
     *  {@link ConcurrentModule}s
     * @see ConcurrentModule#startThreads()
     */
    public void startModuleThreads() {
        areThreadsStarted = true;
        for (Module module : loadedModules) {
            if (module instanceof ConcurrentModule) {
                ((ConcurrentModule)module).startThreads();
            }
        }
    }

    /**
     * Outputs the status of all loaded modules
     * @see Module#log()
     */
    public void logModuleStatus() {
        for (Module module : loadedModules) {
            module.log();
        }
        opMode.telemetry.update();
    }

    /**
     * Unloads all loaded modules
     * @see Module#cleanupModule()
     */
    public void unloadAll() {
        for (Module module : loadedModules) {
            module.cleanupModule();
        }
    }
}
