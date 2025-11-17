package org.firstinspires.ftc.teamcode.shooting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor motor;
    private double power = 0.85;

    public Shooter(HardwareMap hardwareMap, String name, boolean reversed) {
        motor = hardwareMap.get(DcMotor.class, name);
        if (reversed) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void on() {
        motor.setPower(power);
    }

    public void off() {
        motor.setPower(0);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }
}
