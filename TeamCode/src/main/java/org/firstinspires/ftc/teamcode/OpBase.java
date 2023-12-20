package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.modules.ActiveIntake;
import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.PlaneLauncher;
import org.firstinspires.ftc.teamcode.modules.location.OdometryLocalizer;

import java.util.List;

public abstract class OpBase extends OpMode {

    // Globally Declared Sensors

    // Module Classes
    protected OdometryLocalizer driveTrain;
    protected Arm arm;
    protected PlaneLauncher planeLauncher;
	protected ActiveIntake activeIntake;

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
        driveTrain = new OdometryLocalizer(this);
        arm = new Arm(this);
        planeLauncher = new PlaneLauncher(this);
		activeIntake = new ActiveIntake(this);
        telemetry.addLine("Module classes created");

        telemetry.addLine("Successfully initialized hardware!");
        telemetry.update();
    }

    @Override
    public void init() {
        resetRuntime(); // for thread stuff
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            initHardware();
        }
        catch (InterruptedException e) {
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
        arm.startThreads();
    }

    @Override
    public void stop() {
        arm.cleanupModule();
        driveTrain.cleanupModule();
        planeLauncher.cleanupModule();
        telemetry.addLine("Cleanup done!");
        telemetry.update();
    }
}
