package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

import java.util.List;

public abstract class OpBase extends OpMode {

    // Globally Declared Sensors

    // Module Classes
    /**
     * The OpMode's module manager
     */
    private ModuleManager moduleManager;

    /**
     * Gets this OpMode's {@link ModuleManager}
     * @return The module manager for this OpMode
     */
    protected final ModuleManager getModuleManager() {
        return moduleManager;
    }

    // Global Variables

    /**
     * Initializes global hardware and module classes
     * @throws ExceptionInInitializerError The initialization was unable to complete
     */
    public void initHardware() throws ExceptionInInitializerError {
        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach((hub) -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        telemetry.addLine("Lynx modules configured");

        // Motors

        telemetry.addLine("Independent motors registered");
        
        // Init Module classes
        moduleManager = new ModuleManager(this);
        try {
            initModules();
        }
        catch (Throwable inner) {
            throw new ExceptionInInitializerError(inner);
        }
        telemetry.addLine("Module classes created");

        telemetry.addLine("Successfully initialized hardware!");
        telemetry.update();
    }

    /**
     * When overridden by the child class, initializes all modules used by the OpMOde
     */
    protected abstract void initModules();

    @Override
    public void init() {
        resetRuntime(); // for thread stuff
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            initHardware();
        }
        catch (ExceptionInInitializerError e) {
            telemetry.addData("INIT FAILED WITH MESSAGE", e.getMessage());
            telemetry.update();
            RobotLog.ee(getClass().getSimpleName(), e, "FATAL: INIT FAILED WITH EXCEPTION");
            RobotLog.setGlobalErrorMsg(new RuntimeException("Init failed", e), "Failed to initialize OpMode");
            terminateOpModeNow();
        }
    }

    @Override
    public void start() {
        super.start();
        moduleManager.startModuleThreads();
    }

    @Override
    public void stop() {
        super.stop();
        moduleManager.unloadAll();
        telemetry.addLine("Cleanup done!");
        telemetry.update();
    }
}
