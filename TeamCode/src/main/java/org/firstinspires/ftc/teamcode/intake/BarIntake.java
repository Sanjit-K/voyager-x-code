package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BarIntake {
    private DcMotor motor;
    private double power = 0.8;

    public BarIntake(HardwareMap hardwareMap, String name, boolean reversed) {
        motor = hardwareMap.get(DcMotor.class, name);
        if (reversed) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void spinIntake() {
        motor.setPower(power);
    }

    public void spinOuttake() {
        motor.setPower(-0.67);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}
