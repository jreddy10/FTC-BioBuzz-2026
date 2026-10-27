/*
 * BIOBUZZ TELEOP - SETUP AND USE
 *
 * WHAT THIS FILE DOES
 * - This is the driver-controlled OpMode listed on the Driver Station as
 *   "BioBuzz: Drive Test".
 * - Drive motor setup and holonomic drive calculations are in DriveSubsystem.
 * - Intake motor setup and power control are in IntakeSubsystem.
 *
 * BEFORE RUNNING
 * 1. Configure the robot's four drive motors and intake DC motor in the FTC
 *    Robot Controller configuration.
 * 2. The configured names must exactly match these names (including underscores
 *    and capitalization):
 *       front_left_drive
 *       front_right_drive
 *       back_left_drive
 *       back_right_drive
 *       intake_motor
 *    If your configuration uses different names, update the corresponding
 *    strings in DriveSubsystem.java or IntakeSubsystem.java.
 * 3. Confirm that the drive motors are connected to the expected wheels and
 *    check the direction settings in DriveSubsystem.java. Those directions are
 *    only a starting point and depend on the robot's wiring and drivetrain.
 * 4. Keep the robot securely raised so its wheels and intake can move freely.
 *    Keep hands, wires, clothing, and loose objects away from moving parts.
 *
 * DRIVER CONTROLS
 * - Gamepad 1 left stick up/down: drive forward/backward.
 * - Gamepad 1 left stick left/right: strafe.
 * - Gamepad 1 right stick left/right: turn.
 * - Gamepad 2 right trigger: run intake forward.
 * - Gamepad 2 left trigger: run intake in reverse.
 *   If intake direction is opposite to what you expect, reverse the motor
 *   direction in IntakeSubsystem.java after testing safely.
 *
 * BUILD AND RUN
 * - Build/install the TeamCode Robot Controller app using the team's FTC
 *   Android Studio workflow, then connect the Driver Station to the robot.
 * - Select "BioBuzz: Drive Test", press INIT, check initialization/telemetry,
 *   then press START. Press STOP to end the OpMode.
 * - Test one control at a time at low speed before putting the robot on the
 *   floor. This is a drive/intake test program, not a finished game strategy.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/** Driver-controlled entry point. Keep gamepad mapping and telemetry here. */
@TeleOp(name = "BioBuzz: Drive Test", group = "BioBuzz")
public class BioBuzzTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized - verify motor names and directions");
        telemetry.addData("Intake", "Gamepad 2: right trigger in, left trigger reverse");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Gamepad 1: left stick drives and strafes; right stick turns.
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;

            drive.driveRobotCentric(axial, lateral, yaw);
            intake.setPower(intakePower);

            telemetry.addData("Drive", "forward %.2f | strafe %.2f | turn %.2f",
                    axial, lateral, yaw);
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.update();
        }

        drive.stop();
        intake.stop();
    }
}