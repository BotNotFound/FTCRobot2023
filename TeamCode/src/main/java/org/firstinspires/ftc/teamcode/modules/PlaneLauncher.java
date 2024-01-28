package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class PlaneLauncher extends Module {

    private static final double SERVO_POSITION_LAUNCHED = 1;
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
        launcherServo.runIfAvailable(launcher -> launcher.setPosition(SERVO_POSITION_LAUNCHED));
    }

    public boolean hasLaunched() {
        return launcherServo.requireDevice().getPosition() == SERVO_POSITION_LAUNCHED;
    }

    @Override
    public void cleanupModule() {

    }

    @Override
    public void log() {
        launcherServo.runIfAvailable(launcher -> getTelemetry().addData("[Plane Launcher] has launched", hasLaunched()));
    }
}
