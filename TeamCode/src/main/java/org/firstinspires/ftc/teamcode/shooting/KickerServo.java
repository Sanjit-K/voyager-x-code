package org.firstinspires.ftc.teamcode.shooting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KickerServo {
    private Servo servo;

    // Position constants
    private static final double NORMAL_POSITION = 0.5;
    private static final double KICK_POSITION = 0.39;

    public KickerServo(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(NORMAL_POSITION);
    }

    public void kick() {
        servo.setPosition(KICK_POSITION);
    }

    public void normal() {
        servo.setPosition(NORMAL_POSITION);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
}
