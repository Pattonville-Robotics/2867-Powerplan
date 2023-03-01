package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class SlideTuner extends OpMode {

    private DcMotorEx slide1;
    private DcMotorEx slide2;

    private PIDController controller;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;

    public static double f = 0;

    public static int targPos = 0;

    // ticks per degree. ticks per rev divided by full rev
    private final double tpd = (double) 1120 / 360;

    // TIPS FOR TUNING:
    // GO TO ONBOTJAVA SITE [IP]/dash
    // First, tune feedforward. increase until resists gravity
    // Next, tune p until reaching target accurately
    // If arm is oscillating around target pos, increase d very slightly

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide1 = hardwareMap.get(DcMotorEx.class, "motorLinearSlide");
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2 = hardwareMap.get(DcMotorEx.class, "motorLinearSlide2");
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int curPos = slide1.getCurrentPosition();
        double pid = controller.calculate(curPos, targPos);
        double feedForward = Math.cos(Math.toRadians(targPos / tpd)) * f;

        double p = pid + feedForward;

        slide1.setPower(p);
        slide2.setPower(p);

        telemetry.addData("pos ", curPos);
        telemetry.addData("target " , targPos);

    }
}
