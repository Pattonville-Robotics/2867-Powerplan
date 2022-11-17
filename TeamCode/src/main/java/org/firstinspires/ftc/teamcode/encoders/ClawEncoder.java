package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawEncoder {
    private Servo claw;
    private boolean isOpen = true;
    private LinearOpMode linearOp;
    public ClawEncoder(LinearOpMode linearOp) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        hardwareMap.servo.get("claw");
        claw.setPosition(0);
    }

    public void toggleClaw() {
        if (isOpen) {
            claw.setPosition(0);
            isOpen = false;
        } else {
            claw.setPosition(1);
            isOpen = true;
        }
    }
}
