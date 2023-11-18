package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.Grabber;
import org.firstinspires.ftc.teamcode.modules.PlaneLauncher;
import org.firstinspires.ftc.teamcode.modules.location.PIDController;

import java.util.List;

public abstract class OpBase extends OpMode {

    // Globally Declared Sensors

    // Module Classes
    protected PIDController driveTrain;
    protected Arm arm;
    protected Grabber grabber;
    protected PlaneLauncher planeLauncher;

    // Global Variables

    /**
     * Initializes global hardware and module classes
     * @throws InterruptedException The initialization was unable to complete
     */
    public void initHardware() throws InterruptedException {
        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach((hub) -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        telemetry.addLine("Lynx modules configured");

        // Motors

        telemetry.addLine("Independent motors registered");
        
        // Init Module classes
        driveTrain = new PIDController(this);
        grabber = new Grabber(this);
        arm = new Arm(this);
        planeLauncher = new PlaneLauncher(this);
        telemetry.addLine("Module classes created");

        telemetry.addLine("Successfully initialized hardware!");
        telemetry.update();
    }

    @Override
    public void init() {
        resetRuntime(); // for thread stuff
        try {
            initHardware();
        }
        catch (InterruptedException e) {
            telemetry.addData("INIT FAILED WITH MESSAGE", e.getMessage());
            telemetry.update();
            terminateOpModeNow();
        }
    }

    @Override
    public void stop() {
        arm.cleanupModule();
        driveTrain.cleanupModule();
        grabber.cleanupModule();
        planeLauncher.cleanupModule();
        telemetry.addLine("Cleanup done!");
        telemetry.update();
    }
}
