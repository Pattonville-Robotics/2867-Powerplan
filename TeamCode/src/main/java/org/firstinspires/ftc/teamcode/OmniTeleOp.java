package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.ArmEncoder;

import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;

@TeleOp
//@Disabled
public class OmniTeleOp extends LinearOpMode {

    final float slideSpeed = 0.5f;
    double spdMult;

    @Override
    public void runOpMode() throws InterruptedException {

        final GamepadEx controller1 = new GamepadEx(gamepad1);
//        final MecanumEncoder driveTrain = new MecanumEncoder(this);
        final SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
        // linear slide motor
        final ArmEncoder linearSlide = new ArmEncoder(this, "motorLinearSlide", 600);
        final ArmEncoder linearSlide2 = new ArmEncoder(this, "motorLinearSlide2", 600);
        // virtual 4 bar motor
        final ArmEncoder bar = new ArmEncoder(this, "motorBar", 300);
        final ClawEncoder claw = new ClawEncoder(this);
        double x;
        double y;
        double theta;
        double Px;
        double Py;
        // Higher limit -> less precise control, but avoids unintentional inputs due to stick drift
        double xDriftLimit = 0.1;

        // RR stuff
//        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        // assuming the bot ends auto at the center of field facing 90 deg. TODO carry over x,y,heading from auto
        // https://learnroadrunner.com/advanced.html#transferring-pose-between-opmodes
        driveTrain.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driveTrain.update();
            Pose2d myPose = driveTrain.getPoseEstimate();

            /*
            Drive speed multiplier.
            First, get the analogue value of the left trigger and multiply it by 4, then add 1.
            Then, we get the reciprocal of this so speed decreases as the trigger is held.
            We multiplied by 4 to increase the intensity of slowdown when LT is held.
            We add 1 to ensure LT being unpressed means normal speed, and to avoid division by 0.
            */
            spdMult = ( 1.0d / (1.0 + gamepad1.left_trigger*4.0));

            double rx = controller1.getRightX();

            x = controller1.getLeftX();
            y = controller1.getLeftY();
//            x = Math.cos(Math.acos(controller1.getLeftX()) - (Math.PI/4));
//            y = Math.sin(Math.asin(controller1.getLeftY()) - (Math.PI/4));

//            theta = (Math.atan2(y, x)) - (Math.PI / 2);
//
//            Px = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) * (Math.sin(theta + Math.PI / 4));
//            Py = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) * (Math.sin(theta - Math.PI / 4));

            /*
            Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio, but only when
            at least one is out of the range [-1, 1]
            */
            // Wheel movement
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * spdMult;
            double backLeftPower = (y - x + rx) / denominator * spdMult;
            double frontRightPower = (y - x - rx) / denominator * spdMult;
            double backRightPower = (y + x - rx) / denominator * spdMult;
            driveTrain.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);


            // Linear slide face buttons
            if (gamepad2.a) {
                linearSlide.setHeight(ArmEncoder.LinearPosition.ONE, slideSpeed);
                linearSlide2.setHeight(ArmEncoder.LinearPosition.ONE, slideSpeed);
            }
            if (gamepad2.x) {
                linearSlide.setHeight(ArmEncoder.LinearPosition.TWO, slideSpeed);
                linearSlide2.setHeight(ArmEncoder.LinearPosition.TWO, slideSpeed);
            }
            if (gamepad2.y) {
                linearSlide.setHeight(ArmEncoder.LinearPosition.THREE, slideSpeed);
                linearSlide2.setHeight(ArmEncoder.LinearPosition.THREE, slideSpeed);
            }
            // when moving to zero, go at half speed to prevent issues with the slide's string unspooling.
            if (gamepad2.b) {
                linearSlide.setHeight(ArmEncoder.LinearPosition.ZERO, slideSpeed*0.5);
                linearSlide2.setHeight(ArmEncoder.LinearPosition.ZERO, slideSpeed*0.5);
            }

            // Used to reset the "0" point of the slide if it becomes stuck lowering in auto.
            // If it is not reset, the face button positions will be completely inaccurate
            if (gamepad2.back){
                linearSlide.reset();
                linearSlide2.reset();
            }

            // to account for drift (don't raise when just turning) and potential conflict w/ the specific positions above
            // move linear slide with right stick.
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                linearSlide.analogMoveSlide(-gamepad2.right_stick_y);
                linearSlide2.analogMoveSlide(-gamepad2.right_stick_y);
            }
            // move 4 bar w/ left stick.
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                bar.analogMoveSlide(-gamepad2.left_stick_y);
            }

            if (gamepad2.dpad_up){
                linearSlide.fuck();
                linearSlide.fuck();
            }

            if (gamepad2.dpad_down){
                linearSlide.fuck2();
                linearSlide.fuck2();
            }

            // Claw
            if (gamepad2.left_bumper) claw.openClaw();
            if (gamepad2.right_bumper) claw.closeClaw();
//            if (gamepad2.left_bumper) claw.changeClaw(-0.01);
//            if (gamepad2.right_bumper) claw.changeClaw(0.01);

            telemetry.addData("linearSlidePosition: ", linearSlide.getPos());
            telemetry.addData("barPosition: ", bar.getPos());
            telemetry.addData("stickX : ", x);
            telemetry.addData("stickY : ", y);
            telemetry.addData("FL : ", frontLeftPower);
            telemetry.addData("BL : ", backLeftPower);
            telemetry.addData("FR : ", frontRightPower);
            telemetry.addData("BR : ", backRightPower);
            telemetry.addData("pose x : ", myPose.getX());
            telemetry.addData("pose y : ", myPose.getY());
            telemetry.addData("pose heading : ", myPose.getHeading());
            telemetry.addData("clawm : ", claw.getPosition());

            telemetry.update();
        }
    }
}