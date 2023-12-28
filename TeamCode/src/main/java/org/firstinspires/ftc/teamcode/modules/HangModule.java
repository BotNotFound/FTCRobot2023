package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;

public class HangModule extends Module {
    private final ConditionalHardwareDevice<DcMotor> hangMotor;

    public static final String HANG_MOTOR_NAME = "Hang Motor";

    /**
     * Initializes the module and registers it with the specified OpMode.  This is where references to any hardware
     * devices used by the module are loaded.
     *
     * @param registrar The OpMode initializing the module
     * @implNote In order to be used in {@link org.firstinspires.ftc.teamcode.modules.core.ModuleManager}, all modules
     * should have a public constructor that takes exactly the same parameters as this one
     * @see org.firstinspires.ftc.teamcode.modules.core.ModuleManager#getModule(Class)
     */
    public HangModule(OpMode registrar) {
        super(registrar);
        hangMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, HANG_MOTOR_NAME);
    }

    /**
     * Ran by parent OpMode in its stop() method
     * Cleans up items like background threads
     */
    @Override
    public void cleanupModule() {

    }

    /**
     * Logs data about the module to telemetry
     */
    @Override
    public void log() {

    }
}
