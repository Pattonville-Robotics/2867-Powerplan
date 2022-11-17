package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlideEncoder {
    LinearOpMode linearOp;
    DcMotor motorLinearSlide;
    private float height;

    public LinearSlideEncoder (LinearOpMode linearOp){
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motorLinearSlide = hardwareMap.dcMotor.get("motorLinearSlide"); // not sure what the id is
        motorLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public enum LinearPosition {
        ZERO(0), ONE(3500), TWO(5650), THREE(8400), CONE1(200), CONE2(400), CONE3(600);
        private final int ticks;
        LinearPosition(int i) {this.ticks = 1;}
    }

    public float getHeight(){
        return (height);
    }

    public void changeHeight(float inches, float speed) throws InterruptedException {
        float time = 0.5f * Math.abs(inches) * (1/speed);

        // 1 or -1 if inches in pos. or neg.
        float dir = inches/(Math.abs(inches));

        motorLinearSlide.setPower(speed*dir);
        linearOp.wait( (long) (1000 * time) );
        motorLinearSlide.setPower(0);
        height += inches;
    }

//    public void setHeight(float inches, float speed) throws InterruptedException{
//        float target = height - inches;
//        changeHeight(target, speed);
//    }

    public void setHeight(LinearPosition pos, double power) {
        motorLinearSlide.setTargetPosition(pos.ticks);
        motorLinearSlide.setPower(power);
    }

}
