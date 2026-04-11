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
public class TSAFieldCentricOpModeV3 extends LinearOpMode {

    // ===== ARM TUNING =====
    private static final double ARM_MAX_POWER        = 0.4;
    private static final double JOINT_MAX_POWER      = 0.3;
    private static final double SECOND_ARM_MAX_POWER = 0.3;
    private static final double JOINT_EXPO           = 2.0;
    private static final double ARM_EXPO             = 2.0;
    private static final double SECOND_ARM_EXPO      = 2.0;
    private static final double STICK_DEADZONE       = 0.05;

    // ===== DRIVE TUNING =====
    private static final double DRIVE_SPEED = 0.5;

    // ===== CLAW TUNING =====
    private static final double CLAW_OPEN  = 0.52;
    private static final double CLAW_CLOSE = 0.63;

    // ===== RING CLAW TUNING =====
    private static final double RING_CLAW_OPEN  = 0.52;
    private static final double RING_CLAW_CLOSE = 0.58;

    /** Applies an exponential curve to a joystick input, preserving sign. */
    private double expoInput(double raw, double exponent) {
        if (Math.abs(raw) < STICK_DEADZONE) return 0.0;
        return Math.signum(raw) * Math.pow(Math.abs(raw), exponent);
    }

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

        // ===== SECOND ARM =====
        DcMotor secondArm = hardwareMap.dcMotor.get("secondArm");

        // ===== CLAWS =====
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo ringClaw  = hardwareMap.servo.get("ringClaw");

        // Drivetrain directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Joint setup
        uArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Second arm setup
        secondArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // ===== STATE =====
        boolean clawOpen     = false;
        boolean ringClawOpen = false;
        boolean lastA        = false;
        boolean lastB        = false;

        clawServo.setPosition(CLAW_CLOSE);
        ringClaw.setPosition(RING_CLAW_CLOSE);

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
            frontLeftMotor.setPower(DRIVE_SPEED  * (rotY + rotX + rx) / denom);
            backLeftMotor.setPower(DRIVE_SPEED   * (rotY - rotX + rx) / denom);
            frontRightMotor.setPower(DRIVE_SPEED * (rotY - rotX - rx) / denom);
            backRightMotor.setPower(DRIVE_SPEED  * (rotY + rotX - rx) / denom);

            // ===== ARM =====
            double rawArm   = -gamepad2.left_stick_y;
            double armPower = expoInput(rawArm, ARM_EXPO) * ARM_MAX_POWER;
            uArmMotor.setPower(armPower);

            // ===== JOINT =====
            double rawJoint   = gamepad2.left_stick_x;
            double jointPower = expoInput(rawJoint, JOINT_EXPO) * JOINT_MAX_POWER;
            uArmJoint.setPower(jointPower);

            // ===== SECOND ARM =====
            double rawSecondArm   = -gamepad2.right_stick_y;
            double secondArmPower = expoInput(rawSecondArm, SECOND_ARM_EXPO) * SECOND_ARM_MAX_POWER;
            secondArm.setPower(secondArmPower);

            // ===== CLAW TOGGLE (A) =====
            boolean a = gamepad2.a;
            if (a && !lastA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? CLAW_OPEN : CLAW_CLOSE);
            }
            lastA = a;

            // ===== RING CLAW TOGGLE (B) =====
            boolean b = gamepad2.b;
            if (b && !lastB) {
                ringClawOpen = !ringClawOpen;
                ringClaw.setPosition(ringClawOpen ? RING_CLAW_OPEN : RING_CLAW_CLOSE);
            }
            lastB = b;

            // ===== TELEMETRY =====
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

            telemetry.addLine("=== ARM ===");
            telemetry.addData("Main arm",   "%.2f", armPower);
            telemetry.addData("Joint",      "%.2f", jointPower);
            telemetry.addData("Second arm", "%.2f", secondArmPower);

            telemetry.addLine("=== CLAWS ===");
            telemetry.addData("Claw",      clawOpen     ? "OPEN" : "CLOSED");
            telemetry.addData("Ring claw", ringClawOpen ? "OPEN" : "CLOSED");

            telemetry.update();
        }
    }
}