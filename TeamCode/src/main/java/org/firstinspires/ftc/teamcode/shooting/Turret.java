package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private DcMotorImplEx shooterMotor;

    double COUNTS_PER_WHEEL_REV = 28; // External through-bore encoder CPR (1:1 gear ratio)

    private DcMotorImplEx transferMotor;
    private Servo turretServo;
    private AnalogInput turretEncoder;

    private double shooterRPM = 2500.0;
    private double farRPM = 3000.0;
    private double transferPower = 1;

    // PID Coefficients for trackTarget
    public static double Kp = 0.02;
    public static double Ki = 0.0;
    public static double Kd = 0.001;
    public static double kStatic = 0.1;

    private ElapsedTime timer = new ElapsedTime();

    // Configurable offset - Voltage reading when turret is physically at 180 (backward)
    // Tune this! Example: If sensor reads 2.5V at 180 degrees, set this to 2.5
    private final double kP_Shooter = 90.0;
    private final double kI_Shooter = 0.0;
    private final double kD_Shooter = 2.1;
    private final double kF_Shooter = 18.0;

    public Turret(HardwareMap hardwareMap, String shooterName, String turretName, String turretEncoderName,
                  String transferName, boolean shooterReversed, boolean transferReversed) {


        shooterMotor = hardwareMap.get(DcMotorImplEx.class, shooterName);
        shooterMotor.setVelocityPIDFCoefficients(kP_Shooter, kI_Shooter, kD_Shooter, kF_Shooter);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (shooterReversed) {
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretServo = hardwareMap.get(Servo.class, turretName);

        transferMotor = hardwareMap.get(DcMotorImplEx.class, transferName);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (transferReversed) {
            transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        turretEncoder = hardwareMap.get(AnalogInput.class, turretEncoderName);
        timer.reset();

    }


    public void on() {
        double targetTPS = shooterRPM * COUNTS_PER_WHEEL_REV / 60.0;
        shooterMotor.setVelocity(targetTPS);
    }
    public void onFar() {
        double targetTPS = farRPM * COUNTS_PER_WHEEL_REV / 60.0;
        shooterMotor.setVelocity(targetTPS);
    }

    public void off() {
        shooterMotor.setPower(0);
    }

    public void transferOn() {
        transferMotor.setPower(transferPower);
    }

    public void transferOff() {
        transferMotor.setPower(1);
    }

    public void setShooterRPM(double RPM) {
        this.shooterRPM = RPM;
    }

    public double getShooterRPM() {
        return shooterMotor.getVelocity() * 60 / COUNTS_PER_WHEEL_REV;
    }

    public double getSetShooterRPM() {
        return this.shooterRPM;
    }



    public double getTurretVoltage() {
        return turretEncoder.getVoltage();
    }


    public void trackTarget(Pose robotPose, Pose targetPose, int offset) {
        double x = targetPose.getX() - robotPose.getX();
        double y = targetPose.getY() - robotPose.getY();
        double targetAngle = Math.atan2(y, x); // Angle in radians
        targetAngle += Math.toRadians(offset);

        double robotHeading = robotPose.getHeading();
        double desiredRelativeAngle = Math.toDegrees(targetAngle - robotHeading); // Degrees

        // Normalize desired angle to 0-360
        desiredRelativeAngle = normalizeAngle(desiredRelativeAngle);

        desiredRelativeAngle = Math.max(80, Math.min(280, desiredRelativeAngle));


        goToPosition(desiredRelativeAngle);
    }

    public void goToPosition(double targetAngleDegrees) {
        turretServo.setPosition(1 - 4*targetAngleDegrees / 1800);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}
