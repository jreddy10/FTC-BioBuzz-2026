package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Safe autonomous starting point. Add and test the team's game-specific route
 * only after the robot mechanisms and drive measurements are known.
 */
@Autonomous(name = "BioBuzz: Auto Template", group = "BioBuzz")
@Disabled
public class BioBuzzAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);

        telemetry.addData("Status", "Auto template - no movement programmed");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            drive.stop();
            telemetry.addData("Status", "Add a measured BIOBUZZ autonomous routine here");
            telemetry.update();
        }
    }
}