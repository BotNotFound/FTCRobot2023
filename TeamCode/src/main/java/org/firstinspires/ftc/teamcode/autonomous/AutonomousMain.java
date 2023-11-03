package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpBaseLinear;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBaseLinear {

    @Override
    public void runOpMode() {
        try {
            driveTrain.moveAndWait(300, 200, 100);
        }
        catch (InterruptedException e) {
            telemetry.addData("ERROR waiting for movement", e.getMessage());
        }
    }
    
}
