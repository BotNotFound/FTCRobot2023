package org.firstinspires.ftc.teamcode.opmode.autonomous.simple;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.opmode.OpBaseLinear;

@Deprecated
@Disabled
@Autonomous(name = "Autonomous Control", group = "Simple")
public class AutonomousMain extends OpBaseLinear {
    private DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.setVelocity(0,1,0);
        Thread.sleep(1000, 0);
        driveTrain.setVelocity(0,0,0);
//        while (doubleClaw.getClawState() != org.firstinspires.ftc.teamcode.modules.DoubleClaw.ClawState.BOTH_RELEASED) doubleClaw.incrementClawState();
    }

    @Override
    protected void initModules() {
        driveTrain = getModuleManager().getModule(DriveTrain.class);
    }
}
