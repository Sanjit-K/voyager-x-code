package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Spindexer {
    private final DcMotorEx spindexerMotor;
    private AnalogInput analogEncoder;

    // --- PIDF Coefficients ---
    // Start with these. If it oscillates, lower Kp. If it stops short, raise kStatic.
    public static double Kp = 0.007;
    public static double Ki = 0.001;
    public static double Kd = 0.0006;
    public static double kStatic = 0.055; // Minimum power to overcome friction

    // PID state
    private double integralSum = 0.0;
    private double lastMeasuredAngle = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime runtimeTimer = new ElapsedTime();

    private double referenceAngle = 0.0;
    private boolean running = false;

    // Settings
    private double toleranceDegrees = 4.0;
    private double maxTimeSeconds = 0.5;
    private static final double ANALOG_MAX_VOLTAGE = 3.3;

    // Calibration
    private double angleOffsetDegrees = 331.0;

    // Telemetry storage
    private double lastError = 0.0;

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.analogEncoder = hardwareMap.get(AnalogInput.class, analogName);

        spindexerMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // --- Input Processing ---

    public double getAngleFromAnalog() {
        if (analogEncoder == null) return 0.0;
        double v = analogEncoder.getVoltage();
        // Clamp to prevent weird spikes
        if (v < 0) v = 0;
        if (v > ANALOG_MAX_VOLTAGE) v = ANALOG_MAX_VOLTAGE;
        return (v / ANALOG_MAX_VOLTAGE) * 360.0;
    }

    public double getCalibratedAngle() {
        double raw = getAngleFromAnalog();
        return normalizeAngleDegrees(raw + angleOffsetDegrees);
    }

    public void calibrateSetCurrentAsZero() {
        if (analogEncoder == null) return;
        double raw = getAngleFromAnalog();
        angleOffsetDegrees = normalizeAngleDegrees(-raw);
    }


    // --- Control Loop ---

    public void startMoveToAngle(double targetDegrees) {
        referenceAngle = normalizeAngleDegrees(targetDegrees);
        integralSum = 0.0;
        timer.reset();
        runtimeTimer.reset();

        // Seed the last measured angle so derivative doesn't spike on first frame
        lastMeasuredAngle = getCalibratedAngle();

        if (analogEncoder != null) running = true;
    }

    public boolean update() {
        if (!running || analogEncoder == null) return false;

        double currentAngle = getCalibratedAngle();
        double error = smallestAngleDifference(referenceAngle, currentAngle);
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6; // safety

        // 1. Integral Zoning: Only integrate if error is small (prevents windup)
        if (Math.abs(error) < 15.0) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        // 2. Derivative on Measurement: Calculates velocity directly
        // (Avoids "kick" when changing target)
        double velocity = smallestAngleDifference(currentAngle, lastMeasuredAngle) / dt;
        lastMeasuredAngle = currentAngle;

        // 3. Calculate Terms
        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = -Kd * velocity; // Negative because it opposes motion

        // 4. Feedforward (kStatic): Helps overcome friction near target
        double fTerm = 0.0;
        if (Math.abs(error) > toleranceDegrees) {
            fTerm = Math.signum(error) * kStatic;
        }

        double out = pTerm + iTerm + dTerm + fTerm;

        // Clamp
        if (out > 1.0) out = 1.0;
        if (out < -1.0) out = -1.0;

        spindexerMotor.setPower(out);

        // Store for telemetry
        lastError = error;

        // Exit condition
        if (Math.abs(error) <= toleranceDegrees && Math.abs(velocity) < 5.0) {
            spindexerMotor.setPower(0.0);
            running = false;
            return false;
        }

        if (runtimeTimer.seconds() > maxTimeSeconds) {
            spindexerMotor.setPower(0.0);
            running = false;
            return false;
        }

        return true;
    }

    public void stop() {
        running = false;
        spindexerMotor.setPower(0.0);
    }

    public boolean isBusy() { return running; }
    public double getLastError() { return lastError; }

    // --- Helpers ---
    private double normalizeAngleDegrees(double a) {
        double res = a % 360.0;
        if (res < 0) res += 360.0;
        return res;
    }

    private double smallestAngleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180.0) diff -= 360.0;
        while (diff <= -180.0) diff += 360.0;
        return diff;
    }
}