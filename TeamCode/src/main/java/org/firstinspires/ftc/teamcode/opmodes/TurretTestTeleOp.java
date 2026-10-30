package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/** Standalone, reduced-power OpMode for testing turret rotation. */
@TeleOp(name = "BioBuzz: Turret Test", group = "BioBuzz")
public class TurretTestTeleOp extends LinearOpMode {
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized - verify turret_motor configuration");
        telemetry.addData("Control", "Gamepad 1 left stick left/right rotates turret");
        telemetry.addData("Safety", "Reduced power; watch for cable/mechanical limits");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {
                double rotationInput = gamepad1.left_stick_x;
                if (Math.abs(rotationInput) < JOYSTICK_DEADBAND) {
                    rotationInput = 0;
                }

                turret.setPower(rotationInput * MAX_TURRET_POWER);

                telemetry.addData("Turret power", "%.2f", turret.getPower());
                telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }
}