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
        // needs to go to position on start for getPositon to be accurate
        claw.setPosition(0.23);
    }

    public void openClaw() {
        claw.setPosition(0.38);
        isOpen = true;
    }

    public void closeClaw() {
        claw.setPosition(0.20);
        isOpen = false;
    }

    public double getPosition() {
        return (claw.getPosition());
    }

    public void changeClaw(double change) {
        // dont break the servo
        if (change > 0 && claw.getPosition() >= 0.95) { return; }
        if (change < 0 && claw.getPosition() <= 0.05) { return; }
        claw.setPosition(claw.getPosition() + change);




    }
}

