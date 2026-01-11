package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private DcMotorImplEx shooterMotor;

    double COUNTS_PER_WHEEL_REV = 28; // External through-bore encoder CPR (1:1 gear ratio)

    private DcMotorImplEx transferMotor;
    private CRServo turretServo;
    private AnalogInput turretEncoder;

    private double shooterRPM = 2500.0;
    private double farRPM = 3000.0;
    private double transferPower = 1;

    // PID Coefficients for trackTarget
    public static double Kp = 0.02;
    public static double Ki = 0.0;
    public static double Kd = 0.001;
    public static double kStatic = 0.1;
    public static double toleranceDegrees = 2.0;

    // PID Coefficients for goToPosition
    public static double Kp_goToPosition = 0.01;
    public static double Ki_goToPosition = 0.0;
    public static double Kd_goToPosition = 0.001;

    private double lastError = 0.0;
    private double integralSum = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    private double prevServoAngle = 0.0;
    private double currentTurretAngle = 0.0;

    private static final double ANALOG_MAX_VOLTAGE = 3.3;

    // Configurable offset - Voltage reading when turret is physically at 180 (backward)
    // Tune this! Example: If sensor reads 2.5V at 180 degrees, set this to 2.5
    public static double ENCODER_VOLTAGE_AT_180 = 0.152;
    private final double kP_Shooter = 51.5;
    private final double kI_Shooter = 0.0;
    private final double kD_Shooter = 0.0;
    private final double kF_Shooter = 13.45;

    public Turret(HardwareMap hardwareMap, String shooterName, String turretName, String turretEncoderName,
            String transferName, boolean shooterReversed, boolean turretReversed, boolean transferReversed) {


        shooterMotor = hardwareMap.get(DcMotorImplEx.class, shooterName);
        shooterMotor.setVelocityPIDFCoefficients(kP_Shooter, kI_Shooter, kD_Shooter, kF_Shooter);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (shooterReversed) {
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretServo = hardwareMap.get(CRServo.class, turretName);
        if (turretReversed) {
            turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            turretServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        transferMotor = hardwareMap.get(DcMotorImplEx.class, transferName);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (transferReversed) {
            transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        turretEncoder = hardwareMap.get(AnalogInput.class, turretEncoderName);
        timer.reset();

        // Zero tracking on init - assumes turret is physically aligned to 180 degrees
        prevServoAngle = getRawServoAngle();
        currentTurretAngle = 180.0;
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
        transferMotor.setPower(0);
    }

    public void setShooterRPM(double RPM) {
        this.shooterRPM = RPM;
    }

    public double getShooterRPM() {
        return shooterMotor.getVelocity() * 60 / COUNTS_PER_WHEEL_REV;
    }

    public void setTurretPower(double power) {
        turretServo.setPower(power);
    }

    private double getRawServoAngle() {
        double v = turretEncoder.getVoltage();
        if (v < 0)
            v = 0;
        if (v > ANALOG_MAX_VOLTAGE)
            v = ANALOG_MAX_VOLTAGE;
        return (v / ANALOG_MAX_VOLTAGE) * 360.0;
    }

    public double getTurretVoltage() {
        return turretEncoder.getVoltage();
    }

    public void updatePosition() {
        double curr = getRawServoAngle();
        double delta = curr - prevServoAngle;

        // Handle wrap-around (shortest path)
        if (delta < -180) delta += 360;
        else if (delta > 180) delta -= 360;

        prevServoAngle = curr;

        // Accumulate change, applying gear ratio
        // 4:1 ratio means servo moves 4 deg for turret 1 deg
        currentTurretAngle += (delta / 4.0);
    }

    public double getTurretAngle() {
        return currentTurretAngle;
    }

    public void trackTarget(Pose robotPose, Pose targetPose) {
        updatePosition();

        double x = targetPose.getX() - robotPose.getX();
        double y = targetPose.getY() - robotPose.getY();
        double targetAngle = Math.atan2(y, x); // Angle in radians

        double robotHeading = robotPose.getHeading();
        double desiredRelativeAngle = Math.toDegrees(targetAngle - robotHeading); // Degrees

        // Normalize desired angle to 0-360
        desiredRelativeAngle = normalizeAngle(desiredRelativeAngle);

        double currentAngle = getTurretAngle();
        double error = smallestAngleDifference(desiredRelativeAngle, currentAngle);

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0)
            dt = 1e-6;

        // 1. Integral Zoning
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        // 2. Derivative (Standard)
        double derivative = (error - lastError) / dt;

        // 3. Calculate Terms
        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = Kd * derivative;

        // 4. Feedforward (kStatic)
        double fTerm = 0.0;
        if (Math.abs(error) > toleranceDegrees) {
            fTerm = Math.signum(error) * kStatic;
        }

        double power = pTerm + iTerm + dTerm + fTerm;

        // Clamp power
        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;

        turretServo.setPower(power);
        lastError = error;
    }

    public void goToPosition(double targetAngleDegrees) {
        updatePosition();

        double desiredRelativeAngle = normalizeAngle(targetAngleDegrees);

        double currentAngle = getTurretAngle();
        double error = smallestAngleDifference(desiredRelativeAngle, currentAngle);

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0)
            dt = 1e-6;

        // 1. Integral Zoning
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        // 2. Derivative (Standard)
        double derivative = (error - lastError) / dt;

        // 3. Calculate Terms
        double pTerm = Kp_goToPosition * error;
        double iTerm = Ki_goToPosition * integralSum;
        double dTerm = Kd_goToPosition * derivative;

        // 4. Feedforward (kStatic)
        double fTerm = 0.0;
        if (Math.abs(error) > toleranceDegrees) {
            fTerm = Math.signum(error) * kStatic;
        }

        double power = pTerm + iTerm + dTerm + fTerm;

        // Clamp power
        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;

        turretServo.setPower(power);
        lastError = error;
    }



    public void trackLimelight(double tx) {
        updatePosition();
        double error = -tx;
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0)
            dt = 1e-6;

        // 1. Integral Zoning
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        // 2. Derivative (Standard)
        double derivative = (error - lastError) / dt;

        // 3. Calculate Terms
        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = Kd * derivative;

        // 4. Feedforward (kStatic)
        double fTerm = 0.0;
        if (Math.abs(error) > toleranceDegrees) {
            fTerm = Math.signum(error) * kStatic;
        }

        double power = pTerm + iTerm + dTerm + fTerm;

        // Clamp power
        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;

        turretServo.setPower(power);
        lastError = error;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    private double smallestAngleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180)
            diff -= 360;
        while (diff < -180)
            diff += 360;
        return diff;
    }

    public void stop() {
        shooterMotor.setPower(0);
        turretServo.setPower(0);
    }

    public void setTrackedTurretAngle(double angleDegrees) {
        // Set the tracked turret angle (degrees) and resync encoder baseline so deltas continue smoothly
        this.currentTurretAngle = angleDegrees;
        // Reset previous raw reading so next delta is relative to current raw voltage
        this.prevServoAngle = getRawServoAngle();
    }

    public double getTrackedTurretAngle() {
        return this.currentTurretAngle;
    }

    public double getRawServoDegrees() {
        return getRawServoAngle();
    }
}
