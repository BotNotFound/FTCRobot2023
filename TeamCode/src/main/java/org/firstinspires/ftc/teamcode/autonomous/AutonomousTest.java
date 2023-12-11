package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.OpBaseLinear;
import org.firstinspires.ftc.teamcode.modules.location.AprilTagLocator;

@Autonomous(name = "Autonomous Test")
public final class AutonomousTest extends OpBaseLinear {
    private AprilTagLocator aprilTagLocator;

    @Override
    public void initHardware() throws InterruptedException {
        super.initHardware();
        aprilTagLocator = new AprilTagLocator(this);
    }

    @Override
    public void runOpMode() {
        driveTrain.driveToOdometry(Movement.Axis.X.genPointFromAxis(5));
    }

    @Override
    public void stop() {
        super.stop();
        aprilTagLocator.cleanupModule();
    }
}
