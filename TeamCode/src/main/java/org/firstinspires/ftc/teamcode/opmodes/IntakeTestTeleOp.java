package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Standalone driver-controlled OpMode for safely testing the intake motor. */
@TeleOp(name = "BioBuzz: Intake Test", group = "BioBuzz")
public class IntakeTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized - verify intake_motor configuration");
        telemetry.addData("Controls", "Right bumper: forward | Left bumper: reverse");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {
                // Holding both bumpers cancels the forward and reverse commands.
                double intakePower = (gamepad1.right_bumper ? 1.0 : 0.0)
                    - (gamepad1.left_bumper ? 1.0 : 0.0);
                intake.setPower(intakePower);

                telemetry.addData("Intake power", "%.2f", intake.getPower());
                telemetry.update();
            }
        } finally {
            intake.stop();
        }
    }
}