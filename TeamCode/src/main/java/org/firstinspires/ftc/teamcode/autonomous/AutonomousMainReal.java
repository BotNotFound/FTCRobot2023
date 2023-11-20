package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.AprilTagLocator;
import org.firstinspires.ftc.teamcode.modules.location.LocalizedMovement;

@Autonomous(name = "Autonomous Main (Drive To Position)")
public class AutonomousMainReal extends OpBaseLinear {
    @Override
    public void runOpMode() {
        AprilTagLocator aprilTagLocator = new AprilTagLocator(this, 5);
        driveTrain.driveTo(new LocalizedMovement(1,1,0,aprilTagLocator));
    }
}
