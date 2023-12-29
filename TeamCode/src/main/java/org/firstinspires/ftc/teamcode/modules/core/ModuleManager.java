package org.firstinspires.ftc.teamcode.modules.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.modules.*;
import org.firstinspires.ftc.teamcode.modules.concurrent.ConcurrentModule;
import org.firstinspires.ftc.teamcode.modules.detection.PropDetector;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.Odometry;
import org.firstinspires.ftc.teamcode.util.SwitchStatement;

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
     * @throws IllegalArgumentException The provided class is not a registered method
     *  as its only parameter
     * @see Module#Module(OpMode)  Module
     */
    public <T extends Module> T getModule(Class<T> moduleClass) throws IllegalArgumentException {
        if (Modifier.isAbstract(moduleClass.getModifiers())) {
            throw new IllegalArgumentException("Trying to retrieve an instance an abstract class!");
        }

        for (Module module : loadedModules) {
            if (moduleClass.isInstance(module)) {
                return moduleClass.cast(module);
            }
        }

        // no module of the specified type exists
        T module = initModule(moduleClass);
        if (module == null) {
            throw new IllegalArgumentException("Provided class "
                    + moduleClass.getCanonicalName()
                    + " is not a registered module!");
        }

        if (module instanceof ConcurrentModule && areThreadsStarted) {
            ((ConcurrentModule)module).startThreads();
        }

        loadedModules.add(module);
        return module;
    }

    private <T extends Module> T initModule(Class<? extends T> moduleClass) {
        try {
            return moduleClass.cast(
                    new SwitchStatement<Class<?>, Module>(clazz -> null)
                            .addCase(PropDetector.class, clazz -> new PropDetector(opMode))

                            .addCase(AprilTagLocator.class, clazz -> new AprilTagLocator(opMode))
                            .addCase(Odometry.class, clazz -> new Odometry(opMode))

                            .addCase(ActiveIntake.class, clazz -> new ActiveIntake(opMode))
                            .addCase(Arm.class, clazz -> new Arm(opMode))
                            .addCase(DriveTrain.class, clazz -> new DriveTrain(opMode))
                            .addCase(FieldCentricDriveTrain.class, clazz -> new FieldCentricDriveTrain(opMode))
                            .addCase(PlaneLauncher.class, clazz -> new PlaneLauncher(opMode))

                            .execute(moduleClass)
            );
        }
        catch (ClassCastException e) {
            return null; // just in case
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
