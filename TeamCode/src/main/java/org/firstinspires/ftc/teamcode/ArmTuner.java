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
    private PIDController controller2;

    public static double p = 0.01;
    public static double i = 0;
    public static double d = 0;

    public static double f = 0.01;

    public static int targPos = 0;
    private int tester = 0;

    // ticks per degree. ticks per rev divided by full rev
    private final double tpd = (double) 288 / 180.0;

    // TIPS FOR TUNING:
    // GO TO ONBOTJAVA SITE [IP]/dash
    // First, tune feedforward. increase until resists gravity
    // Next, tune p until reaching target accurately
    // If arm is oscillating around target pos, increase d very slightly

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        controller2 = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bar1 = (DcMotorEx) hardwareMap.get("motorBar");
        bar1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bar2 = (DcMotorEx) hardwareMap.get("motorBar2");
        bar2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        controller2.setPID(p, i, d);
        int curPos = bar1.getCurrentPosition();
        double pid = controller.calculate(curPos, targPos);
        double feedForward = Math.cos(Math.toRadians((double) targPos / tpd)) * f;
        int curPos2 = bar2.getCurrentPosition();
        double pid2 = controller.calculate(curPos2, targPos);
        double feedForward2 = Math.cos(Math.toRadians((double) targPos / tpd)) * f;

        double p = pid + feedForward;
        double p2 = pid2 + feedForward2;

        bar1.setPower(p2);
        bar2.setPower(p2);

        tester++;

        telemetry.addData("pos ", curPos);
        telemetry.addData("pos? ", bar1.getCurrentPosition() + "  |  " + bar2.getCurrentPosition());
        telemetry.addData("target " , targPos);
        telemetry.addData("b1p " , bar1.getPower());
        telemetry.addData("b2p " , bar2.getPower());
        telemetry.addData("test ", tester);

    }
}
