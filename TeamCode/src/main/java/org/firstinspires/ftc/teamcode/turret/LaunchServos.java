package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchServos {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final CRServo topServo;

    public LaunchServos(HardwareMap hardwareMap, String leftServoName, String rightServoName, String topServoName) {
        this.leftServo = hardwareMap.get(CRServo.class, leftServoName);
        this.rightServo = hardwareMap.get(CRServo.class, rightServoName);
        this.topServo = hardwareMap.get(CRServo.class, topServoName);
    }

    public void enable(){
        leftServo.setPower(-1);
        rightServo.setPower(1);
        topServo.setPower(-1);
    }

    public void disable(){
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
        topServo.setPower(0.0);
    }
}
