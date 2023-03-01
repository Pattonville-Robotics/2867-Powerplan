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
public class ArmTuner extends OpMode {

    private DcMotorEx bar1;
    private DcMotorEx bar2;

    private PIDController controller;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;

    public static double f = 0;

    public static int targPos = 0;

    // ticks per degree. ticks per rev divided by full rev
    private final double tpd = (double) 288 / 360;

    // TIPS FOR TUNING:
    // GO TO ONBOTJAVA SITE [IP]/dash
    // First, tune feedforward. increase until resists gravity
    // Next, tune p until reaching target accurately
    // If arm is oscillating around target pos, increase d very slightly

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bar1 = hardwareMap.get(DcMotorEx.class, "motorBar");
        bar1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bar2 = hardwareMap.get(DcMotorEx.class, "motorBar2");
        bar2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int curPos = bar1.getCurrentPosition();
        double pid = controller.calculate(curPos, targPos);
        double feedForward = Math.cos(Math.toRadians(targPos / tpd)) * f;

        double p = pid + feedForward;

        bar1.setPower(p);
        bar2.setPower(p);

        telemetry.addData("pos ", curPos);
        telemetry.addData("target " , targPos);

    }
}
