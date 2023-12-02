package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.DoubleClaw;
import org.firstinspires.ftc.teamcode.modules.DriveTrain;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBaseLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        ((DriveTrain)driveTrain).setVelocity(0,-1,0);
        Thread.sleep(1000, 0);
        driveTrain.setVelocity(0,0,0);
        while (doubleClaw.getClawState() != DoubleClaw.ClawState.BOTH_RELEASED) doubleClaw.incrementClawState();
    }
    
}
