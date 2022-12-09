package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.encoders.ClawEncoder;
import org.firstinspires.ftc.teamcode.encoders.LinearSlideEncoder;
import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    // Speed of
    float slideSpeed;
    final double changeConst = 0.0009;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx controller1 = new GamepadEx(gamepad1);
        GamepadEx controller2 = new GamepadEx(gamepad2);

        final MecanumEncoder driveTrain = new MecanumEncoder(this);
        final LinearSlideEncoder linearSlide = new LinearSlideEncoder(this);
        final ClawEncoder claw = new ClawEncoder(this);

        // Set up button readers
        // Linear slide movement
//        GamepadButton aButton = new GamepadButton(controller1, GamepadKeys.Button.A);
//        GamepadButton xButton = new GamepadButton(controller1, GamepadKeys.Button.X);
//        GamepadButton yButtonnn,,./. n  = new GamepadButton(controller1, GamepadKeys.Button.Y);
//        GamepadButton bButton = new GamepadButton(controller1, GamepadKeys.Button.B);

//        ButtonReader aButton = new ButtonReader(controller1, GamepadKeys.Button.A);
//        ButtonReader xButton = new ButtonReader(controller1, GamepadKeys.Button.X);
//        ButtonReader yButton = new ButtonReader(controller1, GamepadKeys.Button.Y);
//        ButtonReader bButton = new ButtonReader(controller1, GamepadKeys.Button.B);

        // Claw
//        GamepadButton leftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
//        ButtonReader leftBumper = new ButtonReader(controller1, GamepadKeys.Button.LEFT_BUMPER);

        // Linear slide movement
//        aButton.whenPressed(() -> linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ONE, slideSpeed));
//        xButton.whenPressed(() -> linearSlide.setHeight(LinearSlideEncoder.LinearPosition.TWO, slideSpeed));
//        yButton.whenPressed(() -> linearSlide.setHeight(LinearSlideEncoder.LinearPosition.THREE, slideSpeed));
//        bButton.whenPressed(() -> linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ZERO, slideSpeed));

        // Claw
//        leftBumper.whenPressed(claw::toggleClaw);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = controller1.getLeftY();
            double x = controller1.getLeftX() * 1.1; // Counteract imperfect strafing
            double rx = controller1.getRightX();
            /*
            Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio, but only when
            at least one is out of the range [-1, 1]
            */
            // Mecanum movement
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            driveTrain.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            // Linear slide speed
            float LT = gamepad1.left_trigger;
            slideSpeed = LT == 0 ? 0.1f : 0.1f*LT;

            // Linear slide
            if (gamepad2.a) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ONE, slideSpeed);
            if (gamepad2.x) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.TWO, slideSpeed);
            if (gamepad2.y) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.THREE, slideSpeed);
            if (gamepad2.b) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ZERO, slideSpeed);

            if (gamepad2.dpad_down) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.CONE1, slideSpeed);
            if (gamepad2.dpad_left) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.CONE2, slideSpeed);
            if (gamepad2.dpad_up) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.CONE3, slideSpeed);
            if (gamepad2.dpad_right) linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ZERO, slideSpeed);

//            if (gamepad2.a) {
//                linearSlide.setHeight(LinearSlideEncoder.LinearPosition.ZERO, slideSpeed);
//                claw.openClaw();
//            }

            // to account for drift (don't raise when just turning) and potential conflict w/ the specific positions above
            if (Math.abs(gamepad2.right_stick_y) > 0.1) linearSlide.analogMoveSlide(-gamepad2.right_stick_y);

            // Claw

            if (gamepad2.left_bumper) claw.changeClaw(changeConst);
            if (gamepad2.right_bumper) claw.changeClaw(-changeConst);

//            if (aButton.get()) telemetry.addLine("A pressed");
//            if (xButton.get()) telemetry.addLine("X pressed");
//            if (yButton.get()) telemetry.addLine("Y pressed");
//            if (bButton.get()) telemetry.addLine("B pressed");

            telemetry.addData("CurrentLsPosition", linearSlide.currentPosition);
            telemetry.update();
        }
    }
}