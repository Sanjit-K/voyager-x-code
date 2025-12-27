package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor shooterMotor;
    private DcMotor turretMotor;
    private double shooterPower = 0.85;
    private double turretPower = 0.5;
    public static double TICKS_PER_DEGREE = 10; // TODO: Tune this value

    public Turret(HardwareMap hardwareMap, String shooterName, String turretName, boolean shooterReversed, boolean turretReversed) {
        shooterMotor = hardwareMap.get(DcMotor.class, shooterName);
        if (shooterReversed) {
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor = hardwareMap.get(DcMotor.class, turretName);
        if (turretReversed) {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void on() {
        shooterMotor.setPower(shooterPower);
    }

    public void off() {
        shooterMotor.setPower(0);
    }

    public void setShooterPower(double power) {
        this.shooterPower = power;
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public void setTurretPower(double power) {
        this.turretPower = power;
    }

    public void trackTarget(Pose robotPose, Pose targetPose) {
        double x = targetPose.getX() - robotPose.getX();
        double y = targetPose.getY() - robotPose.getY();
        double targetAngle = Math.atan2(y, x); // Angle in radians

        double robotHeading = robotPose.getHeading();
        double desiredRelativeAngle = Math.toDegrees(targetAngle - robotHeading); // Degrees

        int currentTicks = turretMotor.getCurrentPosition();
        double currentAngle = currentTicks / TICKS_PER_DEGREE;

        double deltaAngle = desiredRelativeAngle - currentAngle;

        // Normalize deltaAngle to be within [-180, 180]
        while (deltaAngle > 180) deltaAngle -= 360;
        while (deltaAngle <= -180) deltaAngle += 360;

        int targetTicks = (int) (currentTicks + (deltaAngle * TICKS_PER_DEGREE));

        turretMotor.setTargetPosition(targetTicks);
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        turretMotor.setPower(turretPower);
    }

    public void stop() {
        shooterMotor.setPower(0);
        turretMotor.setPower(0);
    }
}
