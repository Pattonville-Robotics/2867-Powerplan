package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    MecanumEncoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumEncoder encoder = new MecanumEncoder(this);
        float speed = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            encoder.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
//            boolean dpU = gamepad1.dpad_up;
//            boolean dpD = gamepad1.dpad_down;
//            boolean dpL = gamepad1.dpad_left;
//            boolean dpR = gamepad1.dpad_right;
//
//
//            if (dpU) {
//                encoder.moveForward(6);
//            } else if (dpD) {
//                encoder.moveForward(-6) {
//            } else if (dpR) {
//                encoder.rotateDegrees(5);
//            } else if (dpL) {
//                encoder.rotateDegrees(-5);
//            }
        }
    }
}