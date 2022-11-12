package org.firstinspires.ftc.teamcode.dependencies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotParameters {
    public static final double wheelRadius = 2.6;
    public static final double wheelBaseRadius = 12.5;
    public static final int ticksPerInch = 1440;

    public static final double wheelCircumference = wheelRadius * 2 * Math.PI;
    public static final double wheelBaseCircumference = wheelBaseRadius * 2 * Math.PI;
}
