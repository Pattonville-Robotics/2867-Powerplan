package org.firstinspires.ftc.teamcode.encoders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniEncoder {
    LinearOpMode linearOp;
    DcMotorEx motorTop;
    DcMotorEx motorBottom;
    DcMotorEx motorLeft;
    DcMotorEx motorRight;
    DcMotorEx[] motorList;

    public OmniEncoder(LinearOpMode linearOp){
        HardwareMap hardwareMap = linearOp.hardwareMap;
        motorTop = (DcMotorEx) hardwareMap.dcMotor.get("Top");
        motorBottom = (DcMotorEx) hardwareMap.dcMotor.get("Bottom");
        motorLeft = (DcMotorEx) hardwareMap.dcMotor.get("Left");
        motorRight = (DcMotorEx) hardwareMap.dcMotor.get("Right");
        motorList = new DcMotorEx[]{motorTop, motorBottom, motorLeft, motorRight};
    }

    public void setPower(double p){
        for (DcMotorEx m : motorList){
            m.setPower(p);
        }
    }

    public void setPower(double pT, double pB, double pL, double pR){
        motorTop.setPower(pT);
        motorBottom.setPower(pB);
        motorLeft.setPower(pL);
        motorRight.setPower(pR);
    }

}
