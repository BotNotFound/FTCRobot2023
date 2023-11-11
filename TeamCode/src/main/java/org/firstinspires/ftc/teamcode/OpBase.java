package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.FieldCentricMovement;
import org.firstinspires.ftc.teamcode.modules.Grabber;
import org.firstinspires.ftc.teamcode.modules.MovementController;

import java.util.List;

public abstract class OpBase extends OpMode {

    // Globally Declared Sensors

    // Module Classes
    protected MovementController mover;
    protected Arm arm;
    protected Grabber grabber;

    protected FieldCentricMovement drive;

    // Global Variables
    protected Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;

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
        mover = new MovementController(this);
        grabber = new Grabber(this);
        arm = new Arm(this);
        telemetry.addLine("Module classes created");
        drive = new FieldCentricMovement();
        telemetry.addLine("Successfully initialized hardware!");
        telemetry.update();
    }

    @Override
    public void init() {
        try {
            initHardware();
        }
        catch (InterruptedException e) {
            telemetry.addData("INIT FAILED WITH MESSAGE", e.getMessage());
            requestOpModeStop();
        }
    }
} 
