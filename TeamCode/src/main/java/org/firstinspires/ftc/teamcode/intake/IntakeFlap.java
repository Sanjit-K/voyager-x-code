package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeFlap {
    public static final double OFF_POSITION = 0.15;
    public static final double ON_POSITION = 0.0;
    public Servo servo;
    public IntakeFlap(HardwareMap hardwareMap, String name){
        servo = hardwareMap.get(Servo.class, name);
    }

    public void off(){
        servo.setPosition(OFF_POSITION);
    }

    public void on(){
        servo.setPosition(ON_POSITION);
    }



}
