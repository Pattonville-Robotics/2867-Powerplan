package org.firstinspires.ftc.teamcode.encoders;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dependencies.RobotParameters;

import java.util.List;

public class MecanumEncoder {
    LinearOpMode linearOp;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    List<DcMotor> motors;

    public MecanumEncoder(LinearOpMode linearOp) {
        this.linearOp = linearOp;

        HardwareMap hardwareMap = linearOp.hardwareMap;
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motors.add(motorFrontLeft);
        motors.add(motorBackLeft);
        motors.add(motorFrontRight);
        motors.add(motorBackRight);

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    public void move(Vector2d direction, double power) throws InterruptedException {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double y = -direction.getY(); // Remember, this is reversed!
        double x = direction.getX() * 1.1; // Counteract imperfect strafing
            /*
            Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio, but only when
            at least one is out of the range [-1, 1]
            */
        // Mecanum movement
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1/power);
        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;
        setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

        double distanceTicks = inchesToTicks(direction.magnitude());
        int frontLeftTicks = (int)(frontLeftPower * distanceTicks * 1/power);
        int backLeftTicks = (int)(frontLeftPower * distanceTicks * 1/power);
        int frontRightTicks = (int)(frontLeftPower * distanceTicks * 1/power);
        int backRightTicks = (int)(frontLeftPower * distanceTicks * 1/power);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setTargetPosition(frontLeftTicks);
        motorBackLeft.setTargetPosition(backLeftTicks);
        motorFrontRight.setTargetPosition(frontRightTicks);
        motorBackRight.setTargetPosition(backRightTicks);

        while (areMotorsBusy() || !motorsReachedTarget(frontLeftTicks, frontRightTicks, backLeftTicks, backRightTicks) && linearOp.opModeIsActive()){
            Thread.yield();
        }

        setPower(0, 0, 0, 0);
        linearOp.sleep(100);
//        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double forwardInches, double rightInches, double power) throws InterruptedException {
        move(new Vector2d(forwardInches, rightInches), power);
    }

    public void moveForward(double inches, double power) throws InterruptedException {
        move(new Vector2d(inches, 0), power);
    }

    public void moveForward(double inches) throws InterruptedException {
        moveForward(inches, 1);
    }

    public void setPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }

    public void setPower(double power) {
        setPower(power, power, power, power);
    }


    public void rotateDegrees(boolean clockwise, double degrees, double power){
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int frontLeftTicks;
        int frontRightTicks;
        int backLeftTicks;
        int backRightTicks;

        double inchesToTravel = degreesToInches(degrees);
        int ticksToTravel = (int) Math.round(inchesToTicks(inchesToTravel));

        if (clockwise) {
            frontLeftTicks = ticksToTravel;
            frontRightTicks = -ticksToTravel;
            backLeftTicks = ticksToTravel;
            backRightTicks = -ticksToTravel;
        } else {
            frontLeftTicks = -ticksToTravel;
            frontRightTicks = ticksToTravel;
            backLeftTicks = -ticksToTravel;
            backRightTicks = ticksToTravel;
        }

        setMotorTargets(frontLeftTicks, frontRightTicks, backLeftTicks, backRightTicks);;

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        while (areMotorsBusy() || !motorsReachedTarget(frontLeftTicks, frontRightTicks, backLeftTicks, backRightTicks) && linearOp.opModeIsActive()){
            Thread.yield();
        }

        setPower(0);
        linearOp.sleep(100);
//        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double degreesToInches(double degrees){
        return (RobotParameters.wheelBaseCircumference) * (degrees/360);
    }
    public double inchesToTicks(double inches){
//        return (int)((inches / this.rP.wheelCircumference) * this.rP.ticks);
        return (inches / RobotParameters.wheelCircumference) * RobotParameters.ticksPerInch;
    }

    public void resetEncoders(){
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean areMotorsBusy() {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) return true;
        }
        return false;
    }

    public int TARGET_REACHED_THRESHOLD = 3;
    protected boolean motorsReachedTarget(int... targetPositions) {
        for (int i = 0; i < 4; i++) {
            if (Math.abs(motors.get(i).getCurrentPosition() - targetPositions[i]) >= TARGET_REACHED_THRESHOLD)
                return false;
        }
        return false;
    }

    protected void setMotorTargets(final int targetPostitionLeft, final int targetPostitionRight){setMotorTargets(targetPostitionLeft, targetPostitionRight, targetPostitionLeft, targetPostitionRight);}

    protected void setMotorTargets(int targetPostitionLeft, int targetPostitionRight, int targetPositionBackLeft, int targetPositionBackRight){
        motorFrontLeft.setTargetPosition(targetPostitionLeft);
        motorFrontRight.setTargetPosition(targetPostitionRight);
        motorBackLeft.setTargetPosition(targetPositionBackLeft);
        motorBackRight.setTargetPosition(targetPositionBackRight);
    }
}
