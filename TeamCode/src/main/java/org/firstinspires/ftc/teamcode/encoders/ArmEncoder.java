// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!
// DO NOT USE THIS ENCODER USE THE ARM ENCODER!!!!!!!!

package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    private LinearPosition currentPosition = LinearPosition.ZERO;
    private float analogPos;

    public ArmEncoder(LinearOpMode linearOp, String motorName) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        this.motor = hardwareMap.dcMotor.get(motorName);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Tested heights for junctions, in motor ticks.
    public enum LinearPosition {
        ZERO(10),
        ONE(1400),
        TWO(2300),
        THREE(3200),
        CONE1(700),
        CONE2(120),
        CONE3(180);
        private final int ticks;
        LinearPosition(int i) {this.ticks = i;}
    }

    public void setHeight(LinearPosition pos, double power) {
        this.motor.setPower(power);
        this.currentPosition = pos;
        this.motor.setTargetPosition(pos.ticks);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void analogMoveSlide(float magnitude) {
        // if slide is going above upper bound (3rd junction height), stop and return early. only stop if slide is moving up.
        if ((this.motor.getCurrentPosition() >= LinearPosition.THREE.ticks) && (magnitude > 0)) return;
        // same but for lower bound
//        if ((motor.getCurrentPosition() <= LinearPosition.ZERO.ticks) && (magnitude < 0)) return;

        // a cap on downward slide movement speed to avoid the string unspooling.
//        magnitude = (float) Math.max(magnitude, -0.2);

        this.motor.setTargetPosition((int) (this.motor.getCurrentPosition() + Math.floor(magnitude * 160)));
        this.motor.setPower(magnitude);
        analogPos = this.motor.getCurrentPosition();

    }

    public void reset() {
        // Reset the motor's "0" position. Only necessary in case the slide string gets stuck going down to make positions enum accurate.
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
