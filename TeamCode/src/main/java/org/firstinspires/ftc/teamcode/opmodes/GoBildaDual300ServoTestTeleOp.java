package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/** Runs three limited-range forward/back sweeps of a positional goBILDA servo. */
@TeleOp(name = "BioBuzz: Dual 300 Servo Test", group = "BioBuzz")
public class GoBildaDual300ServoTestTeleOp extends LinearOpMode {
    private static final String SERVO_NAME = "launch_servo";
    private static final double BACKWARD_POSITION = 0.15;
    private static final double FORWARD_POSITION = 0.85;
    private static final double CENTER_POSITION = 0.50;
    private static final int SWEEP_COUNT = 3;
    private static final long MOVE_TIME_MS = 1200;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo rotationServo = hardwareMap.get(Servo.class, SERVO_NAME);

        telemetry.addData("Servo", SERVO_NAME);
        telemetry.addLine("Configure the goBILDA servo in positional mode.");
        telemetry.addLine("A: run three sweeps | B: cancel after a move");
        telemetry.addLine("Keep the mechanism clear; travel is limited to 15%-85%.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        rotationServo.setPosition(CENTER_POSITION);
        telemetry.addLine("Ready. Press gamepad 1 A to begin.");
        telemetry.update();

        boolean started = false;
        while (opModeIsActive() && !started) {
            Gamepad gamepad = gamepad1;
            started = gamepad != null && gamepad.a;
            if (!started) {
                sleep(20);
            }
        }

        boolean cancelled = false;
        for (int cycle = 1; cycle <= SWEEP_COUNT && opModeIsActive(); cycle++) {
            rotationServo.setPosition(FORWARD_POSITION);
            telemetry.addData("Sweep", "%d/%d forward", cycle, SWEEP_COUNT);
            telemetry.update();
            sleep(MOVE_TIME_MS);
            if (!opModeIsActive() || isCancelPressed()) {
                cancelled = true;
                break;
            }

            rotationServo.setPosition(BACKWARD_POSITION);
            telemetry.addData("Sweep", "%d/%d backward", cycle, SWEEP_COUNT);
            telemetry.update();
            sleep(MOVE_TIME_MS);
            if (!opModeIsActive() || isCancelPressed()) {
                cancelled = true;
                break;
            }
        }

        if (opModeIsActive()) {
            rotationServo.setPosition(CENTER_POSITION);
            telemetry.addData("Status", cancelled ? "Cancelled" : "Complete");
            telemetry.update();
        }
    }

    private boolean isCancelPressed() {
        Gamepad gamepad = gamepad1;
        return gamepad != null && gamepad.b;
    }
}
