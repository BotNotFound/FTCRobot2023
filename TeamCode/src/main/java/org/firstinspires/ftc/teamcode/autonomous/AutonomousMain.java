package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBaseLinear {

    @Override
    public void runOpMode() {

        driveTrain.setVelocity(0,1,0);
    }
    
}
