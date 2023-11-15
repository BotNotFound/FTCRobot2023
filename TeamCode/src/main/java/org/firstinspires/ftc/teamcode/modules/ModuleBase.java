package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class ModuleBase {

    public final OpMode parent;

    /**
     * Initializes the module and registers it with the specified OpMode
     * @param registrar The OpMode initializing the module
     */
    public ModuleBase(@NonNull OpMode registrar) {
        parent = registrar;
    }

    /**
     * Used for logging from modules
     */
    public Telemetry getTelemetry() {
        return parent.telemetry;
    }

    /**
     * Ran by parent OpMode in its stop() method
     */
    public abstract void cleanupModule();
}
