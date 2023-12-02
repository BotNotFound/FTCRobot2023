package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher extends ModuleBase {

    private static final double SERVO_POSITION_LAUNCHED = 0;
    public static final String LAUNCHER_SERVO_NAME = "Launcher Servo";

    /**
     * The servo launching the paper airplane
     */
    private Servo launcherServo;

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public PlaneLauncher(@NonNull OpMode registrar) {
        super(registrar);
        try {
            launcherServo = registrar.hardwareMap.get(Servo.class, LAUNCHER_SERVO_NAME);
            getTelemetry().addLine("Launcher servo found!");
        }
        catch (IllegalArgumentException e) {
            registrar.telemetry.addLine("Could not find launcher servo!  Module not initialized...");
        }
    }

    public void launch() {
        if (launcherServo == null) {
            getTelemetry().addLine("[Plane Launcher] No servo to activate!");
            return;
        }
        getTelemetry().addLine("Launching plane");
        launcherServo.setPosition(SERVO_POSITION_LAUNCHED);
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
