package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo clawServo = hardwareMap.servo.get("clawServo");

        double position = 0.5;
        clawServo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {
            // D-pad up/down to nudge position
            if (gamepad1.dpad_up) {
                position += 0.01;
                sleep(100);
            } else if (gamepad1.dpad_down) {
                position -= 0.01;
                sleep(100);
            }

            // Clamp to valid servo range
            position = Math.max(0.0, Math.min(1.0, position));
            clawServo.setPosition(position);

            telemetry.addData("Position", position);
            telemetry.addData("Controls", "D-pad up/down to move");
            telemetry.update();
        }
    }
}