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
    private static final double ARM_MOVE_POWER = 0.2;
    private static final double ARM_HOLD_POWER = 0.18;

    // ===== CLAW SERVO POSITIONS =====
    private static final double CLAW_CLOSED_POS = 0;
    private static final double CLAW_OPEN_POS   = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVETRAIN =====
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // ===== ARM =====
        DcMotor uArmMotor = hardwareMap.dcMotor.get("uArmMotor");

        // ===== CLAW =====
        Servo clawServo = hardwareMap.servo.get("clawServo");

        // Drivetrain directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // Start closed
        clawServo.setPosition(CLAW_CLOSED_POS);

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

            // ===== ARM CONTROL =====
            if (gamepad1.dpad_right) {
                uArmMotor.setPower(ARM_MOVE_POWER);
            } else if (gamepad1.dpad_left) {
                uArmMotor.setPower(-1.0); // ONLY CHANGE: max power on the way back
            } else {
                uArmMotor.setPower(ARM_HOLD_POWER);
            }

            // ===== CLAW TOGGLE (A BUTTON) =====
            boolean a = gamepad1.a;

            if (a && !lastA) {   // rising edge
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? CLAW_OPEN_POS : CLAW_CLOSED_POS);
            }

            lastA = a;

            telemetry.addData("Claw state", clawOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Claw pos", clawServo.getPosition());
            telemetry.update();
        }
    }
}
