package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchServos {
    private final CRServo leftServo;
    private final CRServo rightServo;

    public LaunchServos(HardwareMap hardwareMap, String leftServoName, String rightServoName) {
        this.leftServo = hardwareMap.get(CRServo.class, leftServoName);
        this.rightServo = hardwareMap.get(CRServo.class, rightServoName);
    }

    public void enable(){
        leftServo.setPower(-1.0);
        rightServo.setPower(1.0);
    }

    public void disable(){
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
    }
}
