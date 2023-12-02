package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;

@Autonomous(name = "Autonomous Control (Non-Backdrop Side)")
public class AutonomousMainFarSide extends OpBaseLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        Thread.sleep(5000, 0);
        driveTrain.resetRotation();
        driveTrain.setVelocity(0,1,0);
        Thread.sleep(3000, 0);
        driveTrain.setVelocity(0,0,0);
//        while (doubleClaw.getClawState() != org.firstinspires.ftc.teamcode.modules.DoubleClaw.ClawState.BOTH_RELEASED) doubleClaw.incrementClawState();
    }
    
}
