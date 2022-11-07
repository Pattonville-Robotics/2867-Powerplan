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
            double lx = gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
            double ly = -gamepad1.left_stick_y;         // Remember, this is reversed!
            boolean lc = gamepad1.left_stick_button;

            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;         // Not necessarily necessary.
            boolean rc = gamepad1.right_stick_button;   //

            boolean dpU = gamepad1.dpad_up;
            boolean dpD = gamepad1.dpad_down;
            boolean dpL = gamepad1.dpad_left;
            boolean dpR = gamepad1.dpad_right;

            if (dpU) {
                encoder.moveForward(6);
            } else if (dpD) {
                encoder.moveForward(-6);
            } else if (dpR) {
                encoder.rotateDegrees(5);
            } else if (dpL) {
                encoder.rotateDegrees(-5);
            }
        }
    }
}