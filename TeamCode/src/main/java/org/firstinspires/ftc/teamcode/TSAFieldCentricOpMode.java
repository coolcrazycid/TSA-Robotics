package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TSAFieldCentricOpMode extends LinearOpMode {

    // ===== ARM TUNING =====
    private static final double ARM_MOVE_POWER = 0.5;
    private static final double JOINT_MOVE_POWER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVETRAIN =====
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // ===== ARM =====
        DcMotor uArmMotor = hardwareMap.dcMotor.get("uArmMotor");

        // ===== ARM JOINT =====
        DcMotor uArmJoint = hardwareMap.dcMotor.get("uArmJoint");

        // ===== CLAW =====
        Servo clawServo = hardwareMap.servo.get("clawServo");

        // Hardcoded for testing
        double clawClosedPos = 0.5;
        double clawOpenPos = 0.4;

        // Drivetrain directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Arm joint setup
        uArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU init
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // ===== CLAW STATE =====
        boolean clawOpen = false;
        boolean lastA = false;

        // Set servo to closed position on start
        clawServo.setPosition(clawClosedPos);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== FIELD-CENTRIC DRIVE =====
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX *= 1.1;

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denom);
            backLeftMotor.setPower((rotY - rotX + rx) / denom);
            frontRightMotor.setPower((rotY - rotX - rx) / denom);
            backRightMotor.setPower((rotY + rotX - rx) / denom);

            // ===== ARM CONTROL (UP/DOWN DPAD) =====
            if (gamepad1.dpad_up) {
                uArmMotor.setPower(ARM_MOVE_POWER);
            } else if (gamepad1.dpad_down) {
                uArmMotor.setPower(-ARM_MOVE_POWER);
            } else {
                uArmMotor.setPower(0.0);
            }

            // ===== ARM JOINT CONTROL (LEFT/RIGHT DPAD) =====
            if (gamepad1.dpad_right) {
                uArmJoint.setPower(JOINT_MOVE_POWER);
            } else if (gamepad1.dpad_left) {
                uArmJoint.setPower(-JOINT_MOVE_POWER);
            } else {
                uArmJoint.setPower(0.0);
            }

            // ===== CLAW TOGGLE (A BUTTON) =====
            boolean a = gamepad1.a;

            if (a && !lastA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? clawOpenPos : clawClosedPos);
            }

            lastA = a;

            telemetry.addData("Claw state", clawOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Claw current pos", clawServo.getPosition());
            telemetry.addData("Arm power", uArmMotor.getPower());
            telemetry.addData("Arm Joint power", uArmJoint.getPower());
            telemetry.addData("dpad_up", gamepad1.dpad_up);
            telemetry.addData("dpad_down", gamepad1.dpad_down);
            telemetry.addData("dpad_right", gamepad1.dpad_right);
            telemetry.addData("dpad_left", gamepad1.dpad_left);
            telemetry.update();
        }
    }
}