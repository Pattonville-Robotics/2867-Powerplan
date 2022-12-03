package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.encoders.MecanumEncoder;

@Autonomous
@Disabled
public class MecanumAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumEncoder encoder = new MecanumEncoder(this);

        waitForStart();

        encoder.move(new Vector2d(5,5),1);
        encoder.rotateDegrees(true,90,1);
        encoder.move(new Vector2d(5,5),1);
        encoder.rotateDegrees(true,-180,1);
        encoder.move(new Vector2d(10,10),1);

    }
}
