package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.BarEncoder;
import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;


@TeleOp
public class RollaTeleOp extends LinearOpMode {

    // constants/declarations go here

    // how fast the V4b motor will move with analogue stick
    private final double armSpd = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        // init, but not started.
        final BarEncoder bar = new BarEncoder(this);
        final MecanumEncoder drive = new MecanumEncoder(this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Note that this loop runs 20 times per second.

            // DIRECT DRIVETRAIN CONTROL
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double topP = (x + rx);
            double bottomP = (-x + rx);
            double leftP = (y + rx);
            double rightP = (-y + rx);

            drive.setPower(topP, bottomP, leftP, rightP);

            // ARMS (V4B AND SLIDE)
            // v4b
            bar.setTargPos(bar.getTargPos() + (int) (gamepad2.left_stick_y * armSpd));
            bar.updatePID();

            // slide


            // telem
            telemetry.addData("topP ", topP);
            telemetry.addData("bottomP ", bottomP);
            telemetry.addData("leftP ", leftP);
            telemetry.addData("rightP ", rightP);

            telemetry.update();
        }
    }

}
