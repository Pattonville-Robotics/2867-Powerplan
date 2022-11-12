package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumEncoder {
    LinearOpMode linearOp;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    public MecanumEncoder(LinearOpMode linearOp) {
        this.linearOp = linearOp;
        HardwareMap hardwareMap = linearOp.hardwareMap;

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveForward(float inches) throws InterruptedException {
        float time = 0.5f*inches;

        setPower(1, 1, 1, 1);

        linearOp.wait((long)(1000 * time));

        setPower(0,0,0,0);
    }

    public void rotateDegrees(float degrees) throws InterruptedException {
        float time = Math.abs(degrees)/90;

        if (degrees > 0) {
            setPower(1, -1, -1, 1);
        } else {
            setPower(-1, 1, 1, -1);
        }

        linearOp.wait((long)(1000 * time));

        setPower(0,0,0,0);
    }

    public void setPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }
}
