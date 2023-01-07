package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

public class LinearSlideEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    public LinearPosition currentPosition = LinearPosition.ZERO;
    public float analogPos;

    public LinearSlideEncoder (LinearOpMode linearOp){
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motor = hardwareMap.dcMotor.get("motorLinearSlide");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // TODO: hone specific heights, especially cone heights.
    public enum LinearPosition {
        ZERO(0),
        ONE(1400),
        TWO(2300),
        THREE(3200),
        CONE1(300),
        CONE2(120),
        CONE3(180);
        private final int ticks;
        LinearPosition(int i) {this.ticks = i;}
    }

//    public float getHeight(){
//        return (height);
//    }

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
        double speedMult;
//        motorLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLinearSlide.setPower(power);
//        motorLinearSlide.setTargetPosition(pos.ticks);
//        motorLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if ((pos - currentPosition) > 0){
//            speedMult = 0.3;
//        }
//        else{ speedMult = 1;}

        motor.setPower(power);

//        motor.setPower(power * speedMult);

        currentPosition = pos;
//        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
//            motor.setTargetPosition(0);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        motor.setTargetPosition(pos.ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        while (motor.isBusy()){
//            Thread.yield(); // lets the rest of teleop run, i think
//        }
//        motor.setPower(0);
    }

    public void analogMoveSlide(float magnitude) {
        // TODO: increase maximum height for new (yet to be fixed) slide configuration.
        // magnitude: direction and speed of movement                                         vvv max height of the slide, in ticks
//        if (motor.getCurrentPosition() >= 100 && magnitude < 0 || motor.getCurrentPosition() <= 4000 &&  magnitude > 0) { // Disallow adding slack when the slide is lowest
            // cap downward speed                                                                                            and over-tightening when at its highest.
            magnitude = (float) Math.max(magnitude, -0.25);
            motor.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 160)));
            motor.setPower(magnitude);
            analogPos = motor.getCurrentPosition();

//        }
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
