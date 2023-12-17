package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;

public class PlaneLauncher extends ModuleBase {

    private static final double SERVO_POSITION_LAUNCHED = 0;
    public static final String LAUNCHER_SERVO_NAME = "Launcher Servo";

    /**
     * The servo launching the paper airplane
     */
    private final ConditionalHardwareDevice<Servo> launcherServo;

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public PlaneLauncher(@NonNull OpMode registrar) {
        super(registrar);
        launcherServo = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, Servo.class, LAUNCHER_SERVO_NAME);
    }

    public void launch() {
        launcherServo.runIfAvailable(launcher -> {
            getTelemetry().addLine("Launching plane");
            launcher.setPosition(SERVO_POSITION_LAUNCHED);
        }, () -> getTelemetry().addLine("[Plane Launcher] No servo to activate!"));
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {

    }
}
