package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.logging.Logger;

public abstract class OpBaseLinear extends OpBase {

    public OpBaseLinear() {
        super();
    }

    /**
     * Unused
     * @see #loop()
     */
    @Override
    public final void init_loop() {
        super.init_loop();
    }

    /**
     * OpMode's start().  Calls {@link #runOpMode()}
     * @see #runOpMode()
     * @see OpMode#start()
     */
    @Override
    public final void start() {
        super.start();
        try {
            runOpMode();
        }
        catch (Throwable th) {
            Logger.getLogger("org.firstinspires.ftc.teamcode.OpModeLinear").throwing("OpModeLinear", "start", th);
            telemetry.addData("ERROR running OpMode", th.getMessage());
            for (StackTraceElement element : th.getStackTrace()) {
                telemetry.addData("Stack Trace", element);
            }
        }
    }

    /**
     * OpMode's loop() method
     * @see OpMode#loop()
     */
    @Override
    public final void loop() {}

    /**
     * Contains the actual code for the OpMode
     * @see LinearOpMode#runOpMode()
     */
    public abstract void runOpMode() throws InterruptedException;
}
