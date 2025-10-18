package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchMotors {
    private final DcMotorEx motorLeft;
    private final DcMotorEx motorRight;

    public LaunchMotors(HardwareMap hardwareMap, Follower follower, String motorLeftName, String motorRightName) {
        this.motorLeft = hardwareMap.get(DcMotorEx.class, motorLeftName);;
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorRight = hardwareMap.get(DcMotorEx.class, motorRightName);
    }

    public void set(double power){
        motorRight.setPower(power);
        motorLeft.setPower(power);
    }

    public void enable(){
        set(1.0);
    }

    public void disable(){
        set(0.0);
    }
}