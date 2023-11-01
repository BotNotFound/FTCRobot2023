package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.OpBase;

@Autonomous(name = "Autonomous Control")
public class AutonomousMain extends OpBase {

    @Override
    public void start() {
        driveTrain.moveAndRotateRobot(0.003, -0.002, 3);
    }

    @Override
    public void loop() {

    }
    
}
