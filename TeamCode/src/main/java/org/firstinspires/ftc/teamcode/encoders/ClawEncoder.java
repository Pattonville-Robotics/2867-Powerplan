package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawEncoder {
    private final Servo claw;
    private boolean isOpen = true;
    // exact positions found with telemetry and testing
    private final double openPosition = 0.38;
    private final double closedPosition = 0.20;
    public ClawEncoder(LinearOpMode linearOp) {
        HardwareMap hardwareMap = linearOp.hardwareMap;
        claw = hardwareMap.servo.get("claw");
        // needs to go to position on start for getPositon to be accurate
        claw.setPosition(closedPosition);
    }

    public void openClaw() {
        claw.setPosition(openPosition);
        isOpen = true;
    }

    public void closeClaw() {
        claw.setPosition(closedPosition);
        isOpen = false;
    }

    public double getPosition() {
        return (claw.getPosition());
    }

    public void changeClaw(double change) {
        // keep the servo within reasonable bounds before changing.
        if (change > 0 && claw.getPosition() >= 0.95) { return; }
        if (change < 0 && claw.getPosition() <= 0.05) { return; }
        claw.setPosition(claw.getPosition() + change);




    }
}

