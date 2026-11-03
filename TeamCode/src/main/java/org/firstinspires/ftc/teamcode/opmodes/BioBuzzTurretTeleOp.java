package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/** Drive OpMode with gamepad 2 left-stick control for turret testing. */
@TeleOp(name = "Drive + Turret Test")
public class BioBuzzTurretTeleOp extends LinearOpMode {
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private TurretSubsystem turret;
    private GoBildaPinpointDriver pinpoint;

    private double currentFL;
    private double currentFR;
    private double currentBL;
    private double currentBR;

    private static final double POWER_STEP = 0.08;
    private static final double DEADZONE = 0.08;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "f_l_dr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "b_l_dr");
        frontRightDrive = hardwareMap.get(DcMotor.class, "f_r_dr");
        backRightDrive = hardwareMap.get(DcMotor.class, "b_r_dr");
        turret = new TurretSubsystem(hardwareMap);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeftDrive,
                backLeftDrive,
                frontRightDrive,
                backRightDrive,
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(7, 7, DistanceUnit.CM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        try {
            while (opModeInInit()) {
                pinpoint.update();
                telemetry.addLine("FIELD-RELATIVE DRIVE + TURRET TEST");
                telemetry.addLine("Keep robot stationary during calibration");
                telemetry.addLine("Center turret before INIT; encoder zero is set at INIT");
                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                addPinpointTelemetry();
                telemetry.update();
                sleep(20);
            }

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                // Read gamepads only during the OpMode loop and treat missing input
                // as neutral so a temporarily unavailable controller cannot crash.
                Gamepad driveGamepad = gamepad1;
                Gamepad operatorGamepad = gamepad2;

                double turretInput = operatorGamepad == null
                        ? 0.0 : operatorGamepad.left_stick_x;
                double turretPower = Math.abs(turretInput) < DEADZONE
                        ? 0.0 : turretInput;
                double appliedTurretPower = turret.setPower(turretPower);

                telemetry.addData("Turret input (gamepad 2 left X)", "%.2f", turretInput);
                telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngleDegrees());
                telemetry.addData("Turret commanded power", "%.2f", appliedTurretPower);

                pinpoint.update();

                if (pinpoint.getDeviceStatus()
                        != GoBildaPinpointDriver.DeviceStatus.READY) {
                    stopDrive();
                    telemetry.addData("Pinpoint not ready", pinpoint.getDeviceStatus());
                    telemetry.update();
                    sleep(20);
                    continue;
                }

                double heading = pinpoint.getPosition()
                        .getHeading(AngleUnit.RADIANS);
                if (Double.isNaN(heading) || Double.isInfinite(heading)) {
                    stopDrive();
                    telemetry.addLine("Invalid Pinpoint heading");
                    telemetry.update();
                    sleep(20);
                    continue;
                }

                double y = driveGamepad == null
                        ? 0.0 : -driveGamepad.left_stick_y;
                double x = driveGamepad == null
                        ? 0.0 : driveGamepad.left_stick_x;
                double yaw = driveGamepad == null
                        ? 0.0 : driveGamepad.right_stick_x;

                if (Math.abs(y) < DEADZONE) y = 0;
                if (Math.abs(x) < DEADZONE) x = 0;
                if (Math.abs(yaw) < DEADZONE) yaw = 0;

                if (x == 0 && y == 0 && yaw == 0) {
                    stopDrive();
                } else {
                    double rotX = x * Math.cos(heading) + y * Math.sin(heading);
                    double rotY = -x * Math.sin(heading) + y * Math.cos(heading);

                    double fl = rotY + rotX + yaw;
                    double fr = rotY - rotX - yaw;
                    double bl = rotY - rotX + yaw;
                    double br = rotY + rotX - yaw;

                    double max = Math.max(
                            1.0,
                            Math.max(
                                    Math.max(Math.abs(fl), Math.abs(fr)),
                                    Math.max(Math.abs(bl), Math.abs(br))
                            )
                    );

                    fl /= max;
                    fr /= max;
                    bl /= max;
                    br /= max;

                    currentFL = moveTowards(currentFL, fl);
                    currentFR = moveTowards(currentFR, fr);
                    currentBL = moveTowards(currentBL, bl);
                    currentBR = moveTowards(currentBR, br);

                    frontLeftDrive.setPower(currentFL);
                    frontRightDrive.setPower(currentFR);
                    backLeftDrive.setPower(currentBL);
                    backRightDrive.setPower(currentBR);
                }

                telemetry.addLine("FIELD-RELATIVE DRIVE + TURRET TEST");
                addPinpointTelemetry();
                telemetry.update();
                sleep(20);
            }
        } finally {
            stopDrive();
            turret.stop();
        }
    }

    private void addPinpointTelemetry() {
        telemetry.addData(
                "Heading (degrees)",
                "%.1f",
                pinpoint.getPosition().getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData(
                "X (mm)",
                "%.1f",
                pinpoint.getPosition().getX(DistanceUnit.MM)
        );
        telemetry.addData(
                "Y (mm)",
                "%.1f",
                pinpoint.getPosition().getY(DistanceUnit.MM)
        );
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
        double difference = target - current;
        if (Math.abs(difference) <= POWER_STEP) {
            return target;
        }
        return current + Math.copySign(POWER_STEP, difference);
    }
}
