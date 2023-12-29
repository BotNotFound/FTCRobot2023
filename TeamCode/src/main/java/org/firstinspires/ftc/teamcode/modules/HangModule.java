package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.concurrent.ConcurrentModule;
import org.firstinspires.ftc.teamcode.modules.concurrent.ModuleThread;

import java.util.concurrent.atomic.AtomicBoolean;

public class HangModule extends ConcurrentModule {
    private final AtomicBoolean isUp;

    private final ConditionalHardwareDevice<DcMotor> hangMotor;

    public static final String HANG_MOTOR_NAME = "Hang Motor";

    public static final String HANG_THREAD_GROUP_NAME = "Hang Threads";

    private static class HangPIDThread extends ModuleThread<HangModule> {
        public static final String THREAD_NAME = "Hang PID Thread";

        public HangPIDThread(HangModule host) {
            super(host, THREAD_NAME);
        }

        @Override
        public void execute() {
            host.hangMotor.runIfAvailable(hang -> {

            });
        }
    }

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
        super(registrar, HANG_THREAD_GROUP_NAME);
        hangMotor = ConditionalHardwareDevice.tryGetHardwareDevice(parent.hardwareMap, DcMotor.class, HANG_MOTOR_NAME);
        isUp = new AtomicBoolean(false);
        exitSetup();
    }

    @Override
    protected void registerModuleThreads() {
        registerAsyncOperation(new HangPIDThread(this));
    }

    public void toggleHangState() {
        isUp.set(!isUp.get());
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
