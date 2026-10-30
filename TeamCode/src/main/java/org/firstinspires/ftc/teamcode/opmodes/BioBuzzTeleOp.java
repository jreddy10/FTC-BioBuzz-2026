package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive")
public class BioBuzzTeleOp extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotor turret;

    private GoBildaPinpointDriver pinpoint;

    private double currentFL = 0;
    private double currentFR = 0;
    private double currentBL = 0;
    private double currentBR = 0;

    double y = gamepad2.left_stick_x;
    double y_inverted = -y;
    private static final double POWER_STEP = 0.08;
    private static final double DEADZONE = 0.08;

    @Override
    public void runOpMode() {

        frontLeftDrive =
                hardwareMap.get(DcMotor.class, "f_l_dr");
        backLeftDrive =
                hardwareMap.get(DcMotor.class, "b_l_dr");
        frontRightDrive =
                hardwareMap.get(DcMotor.class, "f_r_dr");
        backRightDrive =
                hardwareMap.get(DcMotor.class, "b_r_dr");
        turret =
                hardwareMap.get(DcMotor.class, "turret");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeftDrive,
                backLeftDrive,
                frontRightDrive,
                backRightDrive
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

        // Retained from your original code.
        // Verify offsets and encoder directions for your mounting.
        pinpoint.setOffsets(7, 7, DistanceUnit.CM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Keep the robot stationary during calibration.
        // Point it toward your intended field-forward direction.
        pinpoint.resetPosAndIMU();



        if (y>0) {
            turret.setPower(y);
        }

        if (y_inverted<0) {
            turret.setPower(y_inverted);
        }

        try {
            while (opModeInInit()) {
                pinpoint.update();

                telemetry.addLine("FIELD-RELATIVE DRIVE");
                telemetry.addLine("Keep robot stationary during calibration");
                telemetry.addData(
                        "Pinpoint status", pinpoint.getDeviceStatus()
                );
                addPinpointTelemetry();
                telemetry.update();

                sleep(20);
            }

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                pinpoint.update();

                // Do not drive with an unready heading sensor.
                if (pinpoint.getDeviceStatus()
                        != GoBildaPinpointDriver.DeviceStatus.READY) {
                    stopDrive();

                    telemetry.addData(
                            "Pinpoint not ready",
                            pinpoint.getDeviceStatus()
                    );
                    telemetry.update();

                    sleep(20);
                    continue;
                }

                double heading = pinpoint.getPosition()
                        .getHeading(AngleUnit.RADIANS);

                if (Double.isNaN(heading)
                        || Double.isInfinite(heading)) {
                    stopDrive();

                    telemetry.addLine("Invalid Pinpoint heading");
                    telemetry.update();

                    sleep(20);
                    continue;
                }

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                if (Math.abs(y) < DEADZONE) y = 0;
                if (Math.abs(x) < DEADZONE) x = 0;
                if (Math.abs(yaw) < DEADZONE) yaw = 0;

                if (x == 0 && y == 0 && yaw == 0) {
                    stopDrive();
                } else {
                    // Convert field-relative joystick commands
                    // into robot-relative movement.
                    double rotX =
                            x * Math.cos(heading)
                                    + y * Math.sin(heading);

                    double rotY =
                            -x * Math.sin(heading)
                                    + y * Math.cos(heading);

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

                telemetry.addLine("FIELD-RELATIVE DRIVE");
                addPinpointTelemetry();
                telemetry.update();

                sleep(20);
            }
        } finally {
            stopDrive();
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