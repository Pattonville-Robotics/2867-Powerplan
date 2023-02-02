package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.LinearSlideEncoder;
import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

@TeleOp
public class OmniTeleOp extends LinearOpMode {

    final float slideSpeed = 0.5f;
    double spdMult;

    @Override
    public void runOpMode() throws InterruptedException {

        final GamepadEx controller1 = new GamepadEx(gamepad1);
        final MecanumEncoder driveTrain = new MecanumEncoder(this);
        final LinearSlideEncoder linearSlide = new LinearSlideEncoder(this);
        final ClawEncoder claw = new ClawEncoder(this);
        double x;
        double y;
        // Higher limit -> less precise control, but avoids unintentional inputs due to stick drift
        double xDriftLimit = 0.1;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /*
            Drive speed multiplier.
            First, get the analogue value of the left trigger and multiply it by 4, then add 1.
            Then, we get the reciprocal of this so speed decreases as the trigger is held.
            We multiplied by 4 to increase the intensity of slowdown when LT is held.
            We add 1 to ensure LT being unpressed means normal speed, and to avoid division by 0.
            */
            spdMult = ( 1.0d / (1.0 + gamepad1.left_trigger*4.0));

            // COMMENTED OUT DUE TO WORKING ON NEW DRIVETRAIN
//            double y = (controller1.getLeftY() * Math.abs(controller1.getLeftY()));
//            if (Math.abs(controller1.getLeftX()) > xDriftLimit) {
//                x = controller1.getLeftX() * 1.1 * Math.abs(controller1.getLeftX() * 1.1); // Counteract imperfect strafing
//            }
//            else {
//                x = 0f;
//            }
            double rx = controller1.getRightX();

            x = Math.cos(Math.acos(controller1.getLeftX()) - (Math.PI/4));
            y = Math.sin(Math.asin(controller1.getLeftY()) - (Math.PI/4));

            /*
            Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio, but only when
            at least one is out of the range [-1, 1]
            */
            // Mecanum movement
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * spdMult;
            double backLeftPower = (y - x + rx) / denominator * spdMult;
            double frontRightPower = (y - x - rx) / denominator * spdMult;
            double backRightPower = (y + x - rx) / denominator * spdMult;
            driveTrain.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            // Linear slide face buttons
            if (gamepad2.a) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ONE, slideSpeed);
            if (gamepad2.x) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.TWO, slideSpeed);
            if (gamepad2.y) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.THREE, slideSpeed);
            // when moving to zero, go at half speed to prevent issues with the slide's string unspooling.
            if (gamepad2.b) {linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ZERO, slideSpeed*0.5);}

            // Used to reset the "0" point of the slide if it becomes stuck lowering in auto.
            // If it is not reset, the face button positions will be completely inaccurate
            if (gamepad2.back){
                linearSlide.reset();
            }

            // to account for drift (don't raise when just turning) and potential conflict w/ the specific positions above
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                linearSlide.analogMoveSlide(-gamepad2.right_stick_y);
            }

            // Claw
            if (gamepad2.left_bumper) claw.openClaw();
            if (gamepad2.right_bumper) claw.closeClaw();

            telemetry.addData("linearSlidePosition", linearSlide.motor.getCurrentPosition());
//            telemetry.addData("SpeedMultiplier", spdMult);

            telemetry.update();
        }
    }
}