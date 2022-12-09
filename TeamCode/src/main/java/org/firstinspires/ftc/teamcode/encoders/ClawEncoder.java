package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawEncoder {
    private Servo claw;
    private boolean isOpen = true;
    public ClawEncoder(LinearOpMode linearOp) {
        HardwareMap hardwareMap = linearOp.hardwareMap;
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(0.2);
    }

    public void openClaw() {
        claw.setPosition(0.5);
        isOpen = true;
    }

    public void closeClaw() {
        claw.setPosition(0);
        isOpen = false;
    }

    public void changeClaw(double change) {
        // dont break the servo
//        if ((! (claw.getPosition() > 0.9) && change > 0) && ( ! (claw.getPosition() < 0.1) && change < 0)) {
            claw.setPosition(claw.getPosition() + change);

//        }


    }
}

