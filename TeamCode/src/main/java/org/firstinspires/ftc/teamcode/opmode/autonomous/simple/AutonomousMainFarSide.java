package org.firstinspires.ftc.teamcode.opmode.autonomous.simple;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.opmode.OpBaseLinear;

@Deprecated
@Disabled
@Autonomous(name = "Autonomous Control (Non-Backdrop Side)", group = "Simple")
public class AutonomousMainFarSide extends OpBaseLinear {
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        Thread.sleep(20000, 0);
        driveTrain.setVelocity(0,1,0);
        Thread.sleep(3000, 0);
        driveTrain.setVelocity(0,0,0);
    }

    @Override
    protected void initModules() {
        driveTrain = getModuleManager().getModule(DriveTrain.class);
    }
}
