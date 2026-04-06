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
public class TSAFieldCentricOpModeV2 extends LinearOpMode {

    // ===== ARM TUNING =====
    private static final double ARM_MAX_POWER        = 0.5;
    private static final double JOINT_MAX_POWER      = 0.4;
    private static final double SECOND_ARM_MAX_POWER = 0.4;
    private static final double JOINT_EXPO           = 2.0;
    private static final double ARM_EXPO             = 2.0;
    private static final double SECOND_ARM_EXPO      = 2.0;
    private static final double STICK_DEADZONE       = 0.05;

    // ===== HOLD TUNING =====
    private static final double JOINT_HOLD_POWER      = 0.15;
    private static final double SECOND_ARM_HOLD_POWER = 0.15;
    private static final int    JOINT_DEAD_ZONE       = 2;
    private static final int    SECOND_ARM_DEAD_ZONE  = 2;

    /** Applies an exponential curve to a joystick input, preserving sign. */
    private double expoInput(double raw, double exponent) {
        if (Math.abs(raw) < STICK_DEADZONE) return 0.0;
        return Math.signum(raw) * Math.pow(Math.abs(raw), exponent);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVETRAIN (gamepad1) =====
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // ===== ARM (gamepad2 left stick Y) =====
        DcMotor uArmMotor = hardwareMap.dcMotor.get("uArmMotor");

        // ===== ARM JOINT (gamepad2 left stick X) =====
        DcMotor uArmJoint = hardwareMap.dcMotor.get("uArmJoint");

        // ===== SECOND ARM (gamepad2 right stick Y) =====
        DcMotor secondArm = hardwareMap.dcMotor.get("secondArm");

        // ===== CLAW (gamepad2 A button) =====
        Servo clawServo = hardwareMap.servo.get("clawServo");

        double clawClosedPos = 0.5;
        double clawOpenPos   = 0.4;

        // Drivetrain directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm setup
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Joint setup
        uArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Second arm setup
        secondArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        boolean lastA    = false;

        // ===== JOINT STATE =====
        int     jointTargetPos = 0;
        boolean jointHoldMode  = false;

        // ===== SECOND ARM STATE =====
        int     secondArmTargetPos = 0;
        boolean secondArmHoldMode  = false;

        clawServo.setPosition(clawClosedPos);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== FIELD-CENTRIC DRIVE (gamepad1 sticks) =====
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

            // ===== ARM CONTROL (gamepad2 left stick Y, exponential) =====
            double rawArm   = -gamepad2.left_stick_y;
            double armPower = expoInput(rawArm, ARM_EXPO) * ARM_MAX_POWER;
            uArmMotor.setPower(armPower);

            // ===== JOINT CONTROL (gamepad2 left stick X, exponential) =====
            double rawJoint   = gamepad2.left_stick_x;
            double jointPower = expoInput(rawJoint, JOINT_EXPO) * JOINT_MAX_POWER;

            boolean jointMoving = Math.abs(rawJoint) > STICK_DEADZONE;

            if (jointMoving) {
                if (jointHoldMode) {
                    uArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    jointHoldMode = false;
                }
                uArmJoint.setPower(jointPower);
            } else {
                if (!jointHoldMode) {
                    jointTargetPos = uArmJoint.getCurrentPosition();
                    uArmJoint.setTargetPosition(jointTargetPos);
                    uArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    uArmJoint.setPower(JOINT_HOLD_POWER);
                    jointHoldMode = true;
                }
                int jointError = Math.abs(uArmJoint.getCurrentPosition() - jointTargetPos);
                if (jointError <= JOINT_DEAD_ZONE) {
                    uArmJoint.setPower(0.1);
                }
            }

            // ===== SECOND ARM CONTROL (gamepad2 right stick Y, exponential) =====
            double rawSecondArm   = -gamepad2.right_stick_y;
            double secondArmPower = expoInput(rawSecondArm, SECOND_ARM_EXPO) * SECOND_ARM_MAX_POWER;

            boolean secondArmMoving = Math.abs(rawSecondArm) > STICK_DEADZONE;

            if (secondArmMoving) {
                if (secondArmHoldMode) {
                    secondArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    secondArmHoldMode = false;
                }
                secondArm.setPower(secondArmPower);
            } else {
                if (!secondArmHoldMode) {
                    secondArmTargetPos = secondArm.getCurrentPosition();
                    secondArm.setTargetPosition(secondArmTargetPos);
                    secondArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    secondArm.setPower(SECOND_ARM_HOLD_POWER);
                    secondArmHoldMode = true;
                }
                int secondArmError = Math.abs(secondArm.getCurrentPosition() - secondArmTargetPos);
                if (secondArmError <= SECOND_ARM_DEAD_ZONE) {
                    secondArm.setPower(0.1);
                }
            }

            // ===== CLAW TOGGLE (gamepad2 A button) =====
            boolean a = gamepad2.a;
            if (a && !lastA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? clawOpenPos : clawClosedPos);
            }
            lastA = a;

            // ===== TELEMETRY =====
            telemetry.addData("--- DRIVE (GP1) ---",           "");
            telemetry.addData("Heading (deg)",                  Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            telemetry.addData("--- ARM (GP2 L-Stick Y) ---",   "");
            telemetry.addData("Arm raw input",                  rawArm);
            telemetry.addData("Arm power",                      armPower);
            telemetry.addData("--- JOINT (GP2 L-Stick X) ---", "");
            telemetry.addData("Joint raw input",                rawJoint);
            telemetry.addData("Joint power",                    uArmJoint.getPower());
            telemetry.addData("Joint mode",                     jointHoldMode ? "HOLDING" : "MOVING");
            telemetry.addData("Joint pos",                      uArmJoint.getCurrentPosition());
            telemetry.addData("--- 2ND ARM (GP2 R-Stick Y) ---", "");
            telemetry.addData("2nd Arm raw input",              rawSecondArm);
            telemetry.addData("2nd Arm power",                  secondArm.getPower());
            telemetry.addData("2nd Arm mode",                   secondArmHoldMode ? "HOLDING" : "MOVING");
            telemetry.addData("2nd Arm pos",                    secondArm.getCurrentPosition());
            telemetry.addData("--- CLAW (GP2 A) ---",          "");
            telemetry.addData("Claw state",                     clawOpen ? "OPEN" : "CLOSED");
            telemetry.update();
        }
    }
}