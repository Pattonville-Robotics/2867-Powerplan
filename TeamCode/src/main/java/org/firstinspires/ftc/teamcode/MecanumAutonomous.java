package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import encoders.MecanumEncoder;

@Autonomous
public class MecanumAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumEncoder encoder = new MecanumEncoder(this);

        waitForStart();

        encoder.moveForward(5);
        encoder.rotateDegrees(90);
        encoder.moveForward(5);
        encoder.rotateDegrees(-180);
        encoder.moveForward(10);
    }
}
