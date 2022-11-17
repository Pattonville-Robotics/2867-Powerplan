package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.encoders.LinearSlideEncoder;
import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gamepad = new GamepadEx(gamepad1);
//        final LinearSlideEncoder sEncoder = new LinearSlideEncoder(this);
        final MecanumEncoder mEncoder = new MecanumEncoder(this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad.getLeftY(); // Remember, this is reversed!
            double x = gamepad.getLeftX() * 1.1; // Counteract imperfect strafing
            double rx = gamepad.getRightX();
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
            mEncoder.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            // Linear slide speed
            ButtonReader rB = new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
            ButtonReader lB = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
            float slideSpeed = (float) 1;
            if (lB.wasJustPressed() && slideSpeed > 0.05) {
                slideSpeed -= 0.05;
            }
            if (rB.wasJustPressed() && slideSpeed < 2) {
                slideSpeed += 0.05;
            }
            // Linear slide movement
//            GamepadButton dpU = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
//            GamepadButton dpD = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
//            if (dpU.get()) {
//                sEncoder.changeHeight(1, slideSpeed);
//            }
//            if (dpD.get()) {
//                sEncoder.changeHeight(-1, slideSpeed);
//            }
//            if (!(dpU.get() || dpD.get())) {
//                sEncoder.changeHeight(0, 1);
//                // Potentially redundant; stops the slide from moving when neither D-Pad buttons is pressed. (THEORETICALLY)
//            }

            // Claw
            ButtonReader a = new ButtonReader(gamepad, GamepadKeys.Button.A);


        }
    }
}