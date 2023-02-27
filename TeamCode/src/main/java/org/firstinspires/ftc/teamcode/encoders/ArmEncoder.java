// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!

package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    private ArmPosition currentPosition = ArmPosition.LS_ZERO;
    private float analogPos;
    private int speed;

    public ArmEncoder(LinearOpMode linearOp, String motorName, int speed) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        this.motor = hardwareMap.dcMotor.get(motorName);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.speed = speed;

    }
    // (Un)Tested heights for junctions, in motor ticks.
    public enum ArmPosition {
        LS_ZERO(10),
        LS_ONE(100),
        LS_TWO(300),
        LS_THREE(500),
        LS_CONE1(30),
        LS_CONE2(60),
        LS_CONE3(90),
        BAR_TEST1(133),
        BAR_TEST2(54),
        BAR_MIN(3),
        BAR_MID(98),
        BAR_MAX(183);
        private final int ticks;
        ArmPosition(int i) {this.ticks = i;}
    }

    public void setHeight(ArmPosition pos, double power) {
        this.motor.setPower(power);
        this.currentPosition = pos;
        this.motor.setTargetPosition(pos.ticks);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateBarSpeed(){
        // we need to ungravity
        double p = 0d;
        int curPos = this.motor.getCurrentPosition();
        int targPos = this.motor.getTargetPosition();
        int mid = ArmPosition.BAR_MID.ticks;
        int min = ArmPosition.BAR_MIN.ticks;
        int max = ArmPosition.BAR_MAX.ticks;

        if (this.motor.getCurrentPosition() < ArmPosition.BAR_MID.ticks){
            // bar is now below midpoint, so moving "up" is against grav
            // BEWARE this is all integer division
            p = (double) (targPos-curPos) / (mid-min);

        }
        else if (curPos < ArmPosition.BAR_MID.ticks) {
            // bar is now below midpoint, so moving "up" is with grav
            p = (double) (curPos-targPos) / (max-mid);

        }
        else {
            // bar is exactly at midpoint
            p = 0.05;
        }

        this.motor.setPower(Math.abs(p));

    }

    public void analogMoveSlide(float magnitude) {
        // if slide is going above upper bound (3rd junction height), stop and return early. only stop if slide is moving up.
//        if ((this.motor.getCurrentPosition() >= LinearPosition.THREE.ticks) && (magnitude > 0)) return;
        // same but for lower bound
//        if ((motor.getCurrentPosition() <= LinearPosition.ZERO.ticks) && (magnitude < 0)) return;

        // a cap on downward slide movement speed to avoid the string unspooling.
//        magnitude = (float) Math.max(magnitude, -0.2);

        this.motor.setTargetPosition((int) (this.motor.getCurrentPosition() + Math.floor(magnitude * this.speed)));
        this.motor.setPower(magnitude);
        analogPos = this.motor.getCurrentPosition();

    }

    public void analogSetBar(float magnitude) {
        this.motor.setPower(magnitude);
        analogPos = this.motor.getCurrentPosition();
    }

    public void reset() {
        // Reset the motor's "0" position. Only necessary in case the slide string gets stuck going down to make positions enum accurate.
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPos(){
        return this.motor.getCurrentPosition();
    }

    public void fuck(){
        this.motor.setPower(1d);
    }

    public void fuck2(){
        this.motor.setPower(0d);
    }

}
