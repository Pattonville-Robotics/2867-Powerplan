package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import encoders.MecanumEncoder;
import encoders.LinearSlideEncoder;
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
            double ly = -gamepad1.left_stick_y;         // Remember, this is reversed!
            boolean lc = gamepad1.left_stick_button;

            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;         // Not necessarily necessary.
            boolean rc = gamepad1.right_stick_button;

            boolean dpU = gamepad1.dpad_up;
            boolean dpD = gamepad1.dpad_down;

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;



        }
    }
}