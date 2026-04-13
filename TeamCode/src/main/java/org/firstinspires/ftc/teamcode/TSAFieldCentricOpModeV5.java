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
public class TSAFieldCentricOpModeV5 extends LinearOpMode {

    // ===== ARM TUNING =====
    private static final double ARM_MAX_POWER        = 0.65;
    private static final double JOINT_MAX_POWER      = -0.4;
    private static final double SECOND_ARM_MAX_POWER = 0.4;

    // ===== DRIVE TUNING =====
    private static final double DRIVE_SPEED = 0.5;

    // ===== CLAW TUNING =====
    private static final double CLAW_OPEN  = 0.52;
    private static final double CLAW_CLOSE = 0.64;

    // ===== RING CLAW TUNING =====
    private static final double RING_CLAW_OPEN  = 0.52;
    private static final double RING_CLAW_CLOSE = 0.58;

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

        // Drivetrain brake mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm setup
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Joint setup
        uArmJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Second arm setup
        secondArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU init
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // State/boolean set up
        boolean clawOpen     = false;
        boolean ringClawOpen = false;
        boolean lastA_gp2    = false;
        boolean lastB        = false;
        boolean lastA_gp1    = false;

        clawServo.setPosition(CLAW_CLOSE);
        ringClaw.setPosition(RING_CLAW_CLOSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //FIELD-CENTRIC DRIVE
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (gamepad1.back) imu.resetYaw();

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX *= 1.1;

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower(DRIVE_SPEED  * (rotY + rotX + rx) / denom);
            backLeftMotor.setPower(DRIVE_SPEED   * (rotY - rotX + rx) / denom);
            frontRightMotor.setPower(DRIVE_SPEED * (rotY - rotX - rx) / denom);
            backRightMotor.setPower(DRIVE_SPEED  * (rotY + rotX - rx) / denom);

            //ARM (triggers)
            double rawArm   = gamepad2.right_trigger - gamepad2.left_trigger;
            double armPower = rawArm * ARM_MAX_POWER;
            uArmMotor.setPower(armPower);

            //JOINT (left stick Y)
            double rawJoint   = -gamepad2.left_stick_y;
            double jointPower = rawJoint * JOINT_MAX_POWER;
            uArmJoint.setPower(jointPower);

            //SECOND ARM
            double rawSecondArm   = -gamepad2.right_stick_y;
            double secondArmPower = rawSecondArm * SECOND_ARM_MAX_POWER;

            if (secondArmPower == 0.0) {
                if (gamepad1.left_bumper) {
                    secondArmPower = SECOND_ARM_MAX_POWER;
                } else if (gamepad1.right_bumper) {
                    secondArmPower = -SECOND_ARM_MAX_POWER;
                }
            }
            secondArm.setPower(secondArmPower);

            //CLAW TOGGLE (gamepad2 A)
            boolean a_gp2 = gamepad2.a;
            if (a_gp2 && !lastA_gp2) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? CLAW_OPEN : CLAW_CLOSE);
            }
            lastA_gp2 = a_gp2;

            //RING CLAW TOGGLE
            boolean b     = gamepad2.b;
            boolean a_gp1 = gamepad1.a;

            if ((b && !lastB) || (a_gp1 && !lastA_gp1)) {
                ringClawOpen = !ringClawOpen;
                ringClaw.setPosition(ringClawOpen ? RING_CLAW_OPEN : RING_CLAW_CLOSE);
            }
            lastB     = b;
            lastA_gp1 = a_gp1;

            //TELEMETRY
            telemetry.addLine("NAVIGATION");
            telemetry.addData("Field heading", "%.1f deg", Math.toDegrees(heading));
            telemetry.addData("Drive power",   "%.2f", Math.hypot(rotX, rotY) * DRIVE_SPEED);

            telemetry.addLine("ARM");
            telemetry.addData("Main arm",   armPower  > 0.05 ? "RAISING" : armPower < -0.05 ? "LOWERING" : "HOLDING");
            telemetry.addData("Joint",      jointPower > 0.05 ? "EXTENDING" : jointPower < -0.05 ? "RETRACTING" : "HOLDING");
            telemetry.addData("Second arm", secondArmPower > 0.05 ? "EXTENDING" : secondArmPower < -0.05 ? "RETRACTING" : "HOLDING");

            telemetry.addLine("PAYLOAD");
            telemetry.addData("Claw (payload)",  clawOpen     ? "OPEN — ready to grab" : "CLOSED — holding");
            telemetry.addData("Ring claw",       ringClawOpen ? "OPEN — ready to grab" : "CLOSED — holding");

            telemetry.update();
        }
    }
}