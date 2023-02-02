package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlideEncoder {
    LinearOpMode linearOp;
    public DcMotor motor;
    public LinearPosition currentPosition = LinearPosition.ZERO;
    public float analogPos;

    public LinearSlideEncoder (LinearOpMode linearOp) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motor = hardwareMap.dcMotor.get("motorLinearSlide");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        motor.setPower(power);
        currentPosition = pos;
        motor.setTargetPosition(pos.ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void analogMoveSlide(float magnitude) {
        // if slide is going above upper bound (3rd junction height), stop and return early. only stop if slide is moving up.
        if ((motor.getCurrentPosition() >= LinearPosition.THREE.ticks) && (magnitude > 0)) return;
        // same but for lower bound
//        if ((motor.getCurrentPosition() <= LinearPosition.ZERO.ticks) && (magnitude < 0)) return;

        // a cap on downward slide movement speed to avoid the string unspooling.
        magnitude = (float) Math.max(magnitude, -0.2);

        motor.setTargetPosition((int) (motor.getCurrentPosition() + Math.floor(magnitude * 160)));
        motor.setPower(magnitude);
        analogPos = motor.getCurrentPosition();
    }

    public void reset() {
        // Reset the motor's "0" position. Only necessary in case the slide string gets stuck going down to make positions enum accurate.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
