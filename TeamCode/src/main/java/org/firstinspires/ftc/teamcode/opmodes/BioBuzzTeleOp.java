package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Drive")
public class BioBuzzTeleOp extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private double currentFL = 0;
    private double currentFR = 0;
    private double currentBL = 0;
    private double currentBR = 0;

    private final double acceleration = 0.08;
    private boolean ballMode = false;

    private static final int BALL_PIPELINE = 0;
    private static final double BALL_STOP_AREA = 8.0;
    private static final double BALL_FORWARD_POWER = 0.35;
    private static final double BALL_TURN_GAIN = 0.025;
    private static final double BALL_MAX_TURN = 0.20;
    private static final double BALL_CENTER_TOLERANCE = 2.0;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(59, 59, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        pinpoint.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(BALL_PIPELINE);
        limelight.start();

        telemetry.addLine("Initialized");
        telemetry.addLine("Field-relative drive ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        while (opModeIsActive()) {
            pinpoint.update();

            if (gamepad1.bWasPressed()) {
                ballMode = !ballMode;
                stopDrive();
                telemetry.addLine(ballMode ? "BALL MODE STARTED" : "BALL MODE STOPPED");
            }

            if (ballMode) {
                runBallTracking();
                addPinpointTelemetry();
                telemetry.update();
                continue;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.08) y = 0;
            if (Math.abs(x) < 0.08) x = 0;
            if (Math.abs(yaw) < 0.08) yaw = 0;

            if (x == 0 && y == 0 && yaw == 0) {
                currentFL = 0;
                currentFR = 0;
                currentBL = 0;
                currentBR = 0;
                stopDrive();
                addPinpointTelemetry();
                telemetry.addLine("FIELD-RELATIVE DRIVE");
                telemetry.update();
                continue;
            }

            double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            double rotX = x * Math.cos(heading) + y * Math.sin(heading);
            double rotY = -x * Math.sin(heading) + y * Math.cos(heading);

            double fl = rotY + rotX + yaw;
            double fr = rotY - rotX - yaw;
            double bl = rotY - rotX + yaw;
            double br = rotY + rotX - yaw;

            double max = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))
            );

            if (max > 1) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            currentFL = moveTowards(currentFL, fl);
            currentFR = moveTowards(currentFR, fr);
            currentBL = moveTowards(currentBL, bl);
            currentBR = moveTowards(currentBR, br);

            frontLeftDrive.setPower(currentFL);
            frontRightDrive.setPower(currentFR);
            backLeftDrive.setPower(currentBL);
            backRightDrive.setPower(currentBR);

            addPinpointTelemetry();
            telemetry.addLine("FIELD-RELATIVE DRIVE");
            telemetry.update();
        }

        stopDrive();
        limelight.stop();
    }

    private void runBallTracking() {
        LLResult result = limelight.getLatestResult();
        telemetry.addLine("BALL MODE");

        if (result == null || !result.isValid()) {
            stopDrive();
            telemetry.addLine("NO LIMELIGHT TARGET");
            return;
        }

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

        if (colorResults == null || colorResults.isEmpty()) {
            stopDrive();
            telemetry.addLine("SEARCHING FOR BALL");
            return;
        }

        LLResultTypes.ColorResult bestTarget = colorResults.get(0);
        for (LLResultTypes.ColorResult target : colorResults) {
            if (target.getTargetArea() > bestTarget.getTargetArea()) {
                bestTarget = target;
            }
        }

        double targetX = bestTarget.getTargetXDegrees();
        double targetArea = bestTarget.getTargetArea();

        telemetry.addData("Ball X", "%.2f degrees", targetX);
        telemetry.addData("Ball Area", "%.2f", targetArea);

        if (targetArea >= BALL_STOP_AREA) {
            stopDrive();
            telemetry.addLine("BALL CLOSE - STOPPED");
            return;
        }

        double turn = 0;
        if (Math.abs(targetX) > BALL_CENTER_TOLERANCE) {
            turn = targetX * BALL_TURN_GAIN;
            turn = Math.max(-BALL_MAX_TURN, Math.min(BALL_MAX_TURN, turn));
        }

        setDrivePowers(
                -BALL_FORWARD_POWER + turn,
                -BALL_FORWARD_POWER - turn,
                -BALL_FORWARD_POWER + turn,
                -BALL_FORWARD_POWER - turn
        );

        telemetry.addLine("LIMELIGHT DRIVING TO BALL");
        telemetry.addData("Turn", "%.3f", turn);
    }

    private void addPinpointTelemetry() {
        double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("X (mm)", "%.1f", pinpoint.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pinpoint.getPosition().getY(DistanceUnit.MM));
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        currentFL = 0;
        currentFR = 0;
        currentBL = 0;
        currentBR = 0;
    }

    private double moveTowards(double current, double target) {
        if (Math.abs(target - current) < acceleration) {
            return target;
        }
        if (target > current) {
            return current + acceleration;
        }
        return current - acceleration;
    }
}
