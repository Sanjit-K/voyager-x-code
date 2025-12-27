package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor motor;
    private double power = 0.85;
    public static double TICKS_PER_DEGREE = 10; // TODO: Tune this value

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

    public void trackTarget(Pose robotPose, Pose targetPose) {
        double x = targetPose.getX() - robotPose.getX();
        double y = targetPose.getY() - robotPose.getY();
        double targetAngle = Math.atan2(y, x); // Angle in radians

        double robotHeading = robotPose.getHeading();
        double turretAngle = targetAngle - robotHeading;

        // Normalize angle to be within -PI to PI
        while (turretAngle > Math.PI) turretAngle -= 2 * Math.PI;
        while (turretAngle <= -Math.PI) turretAngle += 2 * Math.PI;

        int targetPosition = (int) (Math.toDegrees(turretAngle) * TICKS_PER_DEGREE);

        motor.setTargetPosition(targetPosition);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }
}
