package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlideEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    public LinearPosition currentPosition = LinearPosition.ZERO;

    public LinearSlideEncoder (LinearOpMode linearOp){
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motor = hardwareMap.dcMotor.get("motorLinearSlide"); // not sure what the id is
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public enum LinearPosition {
        ZERO(0),
        ONE(1000),
        TWO(1600),
        THREE(2200),
        CONE1(60),
        CONE2(120),
        CONE3(180);
        private final int ticks;
        LinearPosition(int i) {this.ticks = i;}
    }

//    public float getHeight(){
//        return (height);
//    }
//
//    public void changeHeight(float inches, float speed) throws InterruptedException {
//        float time = 0.5f * Math.abs(inches) * (1/speed);
//
//        // 1 or -1 if inches in pos. or neg.
//        float dir = inches/(Math.abs(inches));
//
//        motorLinearSlide.setPower(speed*dir);
//        linearOp.wait( (long) (1000 * time) );
//        motorLinearSlide.setPower(0);
//        height += inches;
//    }

//    public void setHeight(float inches, float speed) throws InterruptedException{
//        float target = height - inches;
//        changeHeight(target, speed);
//    }

    public void setHeight(LinearPosition pos, double power) {
//        motorLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLinearSlide.setPower(power);
//        motorLinearSlide.setTargetPosition(pos.ticks);
//        motorLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentPosition = pos;
//        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
//            motor.setTargetPosition(0);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        motor.setTargetPosition(pos.ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

//        while (motor.isBusy()){
//            Thread.yield(); // lets the rest of teleop run, i think
//        }
//        motor.setPower(0);
    }

    public void analogMoveSlide(float magnitude) {
        // magnitude: direction and speed of movement
        if (magnitude > -0.06 && magnitude < 0.06) { magnitude = 0; }   // accounts for drift, accidental activation when turning.
        if (motor.getCurrentPosition() >= 0 || magnitude > 0) {         // Disallow adding slack when the slide is lowest.
            motor.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 20)));
            motor.setPower(magnitude);
        }
    }

}
