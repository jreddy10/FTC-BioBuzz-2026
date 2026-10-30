package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/** Test OpMode that spins a launcher flywheel to a 6000 RPM target. */
@TeleOp(name = "BioBuzz: Launcher Test", group = "BioBuzz")
public class LauncherTestTeleOp extends LinearOpMode {
    private static final double TARGET_RPM = 6000.0;

    @Override
    public void runOpMode() throws InterruptedException {
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized - verify launcher_motor and encoder setup");
        telemetry.addData("Target", "%.0f RPM", TARGET_RPM);
        telemetry.addData("Controls", "A: toggle flywheel | B: stop");
        telemetry.addData("Safety", "Keep hands and objects clear of the flywheel");
        telemetry.update();

        boolean spinning = false;
        boolean previousA = false;

        try {
            waitForStart();

            while (opModeIsActive()) {
                boolean aPressed = gamepad1.a;
                if (aPressed && !previousA) {
                    spinning = !spinning;
                }
                previousA = aPressed;

                if (gamepad1.b) {
                    spinning = false;
                }

                if (spinning) {
                    launcher.setVelocityRpm(TARGET_RPM);
                } else {
                    launcher.stop();
                }

                telemetry.addData("Launcher", spinning ? "SPINNING" : "STOPPED");
                telemetry.addData("Target RPM", "%.0f", spinning ? TARGET_RPM : 0.0);
                telemetry.addData("Measured RPM", "%.0f", launcher.getVelocityRpm());
                telemetry.update();
            }
        } finally {
            launcher.stop();
        }
    }
}