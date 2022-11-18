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
        claw.setPosition(0);
    }

    public void openClaw() {
        claw.setPosition(1);
        isOpen = true;
    }

    public void closeClaw() {
        claw.setPosition(0);
        isOpen = false;
    }
}
