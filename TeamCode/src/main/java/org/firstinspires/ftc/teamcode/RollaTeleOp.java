package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.BarEncoder;
import org.firstinspires.ftc.teamcode.encoders.LinearSlideEncoder;
import org.firstinspires.ftc.teamcode.encoders.OmniEncoder;

@TeleOp
public class RollaTeleOp extends LinearOpMode {

    // how fast the V4b motor will move with analogue stick
    private final double armSpd = 10;

    double heading;

    @Override
    public void runOpMode() throws InterruptedException {

        // init, but not started.
        final BarEncoder bar = new BarEncoder(this);
        final OmniEncoder drive = new OmniEncoder(this);
        final LinearSlideEncoder slide = new LinearSlideEncoder(this);
        final ClawEncoder claw = new ClawEncoder(this);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Note that this loop runs 20 times per second.

            // DIRECT DRIVETRAIN CONTROL
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // driver centric stuffs
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double driverX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double driverY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double topP = (-driverX + rx);
            double bottomP = (driverX + rx);
            double leftP = (driverY + rx);
            double rightP = (-driverY + rx);

            drive.setPower(topP, bottomP, leftP, rightP);

            // ARMS (V4B AND SLIDE)
            // v4b
            bar.setTargPos(bar.getTargPos() + (int) (gamepad2.left_stick_y * armSpd));
            bar.updatePID();

            // slide
            slide.setPower(gamepad2.right_stick_y);

            // claw
            if (gamepad2.right_bumper){
                if (claw.getPosition()){
                    claw.closeClaw();
                }
                else{
                    claw.openClaw();
                }
            }

            // telem
            telemetry.addData("topP ", topP);
            telemetry.addData("bottomP ", bottomP);
            telemetry.addData("leftP ", leftP);
            telemetry.addData("rightP ", rightP);

            telemetry.update();
        }
    }

}
