package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ActiveIntake extends ModuleBase {
	private final DcMotor intakeMotor;

	public static final String INTAKE_MOTOR_NAME = "Intake Motor";

	public ActiveIntake(OpMode registrar) {
			super(registrar);
			intakeMotor = parent.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
	}

	public void start() {
			intakeMotor.setPower(-1.0);
	}

	public void stop() {
			intakeMotor.setPower(0.0);
	}

	@Override
	public void cleanupModule() {}

	@Override
	public void log() {}
}
