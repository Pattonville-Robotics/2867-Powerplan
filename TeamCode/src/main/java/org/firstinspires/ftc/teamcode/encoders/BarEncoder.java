package org.firstinspires.ftc.teamcode.encoders;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BarEncoder {
    PIDController controller;

    private static DcMotorEx motor1;
    private static DcMotorEx motor2;

    private static final double p = 0.01;
    private static final double i = 0;
    private static final double d = 0;

    public static double f = 0.01;

    public static int targPos = 0;

    // ticks per degree. ticks per rev divided by full rev
    private final double tpd = (double) 288 / 360;

    public BarEncoder(LinearOpMode linearOp){
        controller = new PIDController(p,i,d);
        HardwareMap hardwareMap = linearOp.hardwareMap;

        motor1 = hardwareMap.get(DcMotorEx.class, "motorBar");
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(0);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, "motorBar2");
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setTargetPosition(0);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void updatePID(){
        controller.setPID(p, i, d);
        int curPos = motor1.getCurrentPosition();
        double pid = controller.calculate(curPos, targPos);
        double feedForward = Math.cos(Math.toRadians(targPos / tpd)) * f;

        double p = pid + feedForward;

        motor1.setPower(p);
        motor2.setPower(p);
    }

    public void setTargPos(int t){
        targPos = t;
    }

    public int getTargPos(){
        return targPos;
    }
}
