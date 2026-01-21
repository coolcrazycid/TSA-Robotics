package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TSAFieldCentricOpMode extends LinearOpMode {

    /*
     * ===== U-ARM SETUP =====
     * IMPORTANT: Set this to your motor's encoder ticks per motor revolution.
     * Examples (common, but verify your exact motor/model):
     *  - goBILDA 5202 312RPM: 537.7 ticks/rev
     *  - goBILDA 5202 435RPM: 383.6 ticks/rev
     *  - REV HD Hex (40:1): 1120 ticks/rev
     */
    private static final double UARM_TICKS_PER_MOTOR_REV = 537.7; // <-- CHANGE THIS
    private static final double UARM_GEAR_RATIO = 4.0;            // 4:1 reduction (motor:arm)
    private static final double UARM_DEG_TO_TICKS =
            (UARM_TICKS_PER_MOTOR_REV * UARM_GEAR_RATIO) / 360.0;

    private static final double UARM_TARGET_DEG_90 = 90.0;
    private static final double UARM_TARGET_DEG_0  = 0.0;

    private static final double UARM_RUN_TO_POS_POWER = 0.5;

    private int degToTicks(double degrees) {
        return (int) Math.round(degrees * UARM_DEG_TO_TICKS);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // ===== U-ARM MOTOR (ADD THIS TO YOUR CONFIG) =====
        // In the Robot Configuration, create a motor named exactly: "uArmMotor"
        DcMotor uArmMotor = hardwareMap.dcMotor.get("uArmMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // U-ARM defaults
        // (Flip direction if your arm moves the wrong way)
        uArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        uArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        boolean lastA = false;
        boolean lastB = false;

        while (opModeIsActive()) {
            // ===== DRIVE (FIELD CENTRIC) =====
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ===== U-ARM CONTROL =====
            // Press A -> go to 90 degrees
            // Press B -> go back to 0 degrees
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) {
                int target = degToTicks(UARM_TARGET_DEG_90);
                uArmMotor.setTargetPosition(target);
                uArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                uArmMotor.setPower(UARM_RUN_TO_POS_POWER);
            }

            if (b && !lastB) {
                int target = degToTicks(UARM_TARGET_DEG_0);
                uArmMotor.setTargetPosition(target);
                uArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                uArmMotor.setPower(UARM_RUN_TO_POS_POWER);
            }

            // Optional: once at target, hold with BRAKE + small power (or just 0 with BRAKE)
            if (uArmMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !uArmMotor.isBusy()) {
                uArmMotor.setPower(0.0);
                // If you prefer to switch back:
                // uArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            lastA = a;
            lastB = b;

            // Telemetry to help you verify ticks/angle
            telemetry.addData("UArm encoder", uArmMotor.getCurrentPosition());
            telemetry.addData("UArm target", uArmMotor.getTargetPosition());
            telemetry.addData("UArm mode", uArmMotor.getMode());
            telemetry.update();
        }
    }
}
