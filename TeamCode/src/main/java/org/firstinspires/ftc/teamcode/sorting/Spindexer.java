package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

// Spindexer controlled by PID using an analog 0-3.3V -> 0-360deg sensor.
public class Spindexer {
    private final DcMotorEx spindexerMotor;
    private AnalogInput analogEncoder; // optional, for PID control based on analog 0-3.3V -> 0-360°

    // Max output scaling (software limit). 1.0 = full PID output applied to motor.
    public static double POWER = 1.0;

    // PID gains
    private double Kp = 0.013;
    private double Ki = 0.04;
    private double Kd = 0.0008;

    // PID state
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime runtimeTimer = new ElapsedTime();

    private double referenceAngle = 0.0; // degrees 0-360
    private boolean running = false;
    private double toleranceDegrees = 2.0; // success tolerance
    private double maxIntegral = 1000.0; // anti-windup
    private double maxTimeSeconds = 5.0; // timeout for safety

    private static final double ANALOG_MAX_VOLTAGE = 3.3; // per user's description

    // Calibration parameters
    private double angleOffsetDegrees = 29.0; // add to raw angle after optional reversal
    private boolean angleReversed = false;

    // Telemetry (last computed terms)
    private double lastP = 0.0;
    private double lastI = 0.0;
    private double lastD = 0.0;
    private double lastOutput = 0.0;
    private volatile double lastMeasuredAngle = 0.0;

    // Polling thread (optional). Use startPolling()/stopPolling() to enable continuous background sampling.
    private Thread pollThread = null;
    private volatile boolean polling = false;
    private long pollPeriodMs = 20; // default poll period

    /**
     * Constructor using motor name and optional analog input name (pass null or empty if none).
     */
    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
        spindexerMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (analogName != null && !analogName.isEmpty()) {
            try {
                this.analogEncoder = hardwareMap.get(AnalogInput.class, analogName);
            } catch (Exception e) {
                this.analogEncoder = null; // analog not present
            }
        }
        // If an analog encoder is provided, start background polling by default to keep
        // lastMeasuredAngle up-to-date without requiring explicit calls.
        if (this.analogEncoder != null) {
            // start with the default pollPeriodMs
            startPolling(this.pollPeriodMs);
        }
    }

    // ---------------- analog angle helpers / calibration ----------------

    /**
     * Raw angle from analog input [0,360). Does NOT apply calibration offsets or reversal.
     */
    public double getAngleFromAnalog() {
        if (analogEncoder == null) return 0.0;
        double v = analogEncoder.getVoltage();
        if (v < 0) v = 0;
        if (v > ANALOG_MAX_VOLTAGE) v = ANALOG_MAX_VOLTAGE;
        return (v / ANALOG_MAX_VOLTAGE) * 360.0;
    }

    /**
     * Returns the calibrated angle after applying optional reversal and offset, normalized to [0,360).
     */
    public double getCalibratedAngle() {
        double raw = getAngleFromAnalog();
        double adjusted = angleReversed ? normalizeAngleDegrees(360.0 - raw) : raw;
        double calibrated = normalizeAngleDegrees(adjusted + angleOffsetDegrees);
        lastMeasuredAngle = calibrated;
        return calibrated;
    }

    /**
     * Set the current analog reading to correspond to 0 degrees (useful for quick calibration).
     */
    public void calibrateSetCurrentAsZero() {
        if (analogEncoder == null) return;
        double raw = getAngleFromAnalog();
        angleOffsetDegrees = normalizeAngleDegrees(-raw);
    }

    /**
     * Calibrate so that the current analog reading corresponds to the given knownAngle (degrees).
     */
    public void calibrateSetCurrentAs(double knownAngle) {
        if (analogEncoder == null) return;
        double raw = getAngleFromAnalog();
        angleOffsetDegrees = normalizeAngleDegrees(knownAngle - raw);
    }

    public void setAngleOffsetDegrees(double offset) { this.angleOffsetDegrees = normalizeAngleDegrees(offset); }
    public double getAngleOffsetDegrees() { return angleOffsetDegrees; }

    public void setAngleReversed(boolean reversed) { this.angleReversed = reversed; }
    public boolean isAngleReversed() { return angleReversed; }

    // ---------------- PID control API ----------------

    /**
     * Start a non-blocking move to an absolute angle in degrees (0-360). Call update() repeatedly in loop().
     */
    public void startMoveToAngle(double targetDegrees) {
        referenceAngle = normalizeAngleDegrees(targetDegrees);
        integralSum = 0.0;
        lastError = 0.0;
        if (analogEncoder == null) {
            running = false;
            return;
        }
        timer.reset();
        runtimeTimer.reset();
        running = true;
    }

    /**
     * Call periodically from OpMode.loop(). Returns true while still running.
     */
    public boolean update() {
        if (!running || analogEncoder == null) return false;

        double encoderPosition = getCalibratedAngle();
        double error = smallestAngleDifference(referenceAngle, encoderPosition);

        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-6;
        double derivative = (error - lastError) / dt;
        integralSum += error * dt;

        if (integralSum > maxIntegral) integralSum = maxIntegral;
        if (integralSum < -maxIntegral) integralSum = -maxIntegral;

        double pTerm = Kp * error;
        double iTerm = Ki * integralSum;
        double dTerm = Kd * derivative;

        double out = pTerm + iTerm + dTerm;

        // clamp and apply scaling
        if (out > 1.0) out = 1.0;
        if (out < -1.0) out = -1.0;
        spindexerMotor.setPower(out * POWER);

        // telemetry/store
        lastP = pTerm;
        lastI = iTerm;
        lastD = dTerm;
        lastOutput = out * POWER;

        lastError = error;
        timer.reset();

        if (Math.abs(error) <= toleranceDegrees) {
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

    // ---------------- telemetry getters ----------------

    public double getLastError() { return lastError; }
    public double getLastP() { return lastP; }
    public double getLastI() { return lastI; }
    public double getLastD() { return lastD; }
    public double getLastOutput() { return lastOutput; }
    public double getLastMeasuredAngle() { return lastMeasuredAngle; }

    public double getRawVoltage() { return analogEncoder == null ? 0.0 : analogEncoder.getVoltage(); }

    // ---------------- tuning helpers ----------------
    public void setPID(double kp, double ki, double kd) { this.Kp = kp; this.Ki = ki; this.Kd = kd; }
    public double getKp() { return Kp; }
    public double getKi() { return Ki; }
    public double getKd() { return Kd; }
    public void setToleranceDegrees(double tol) { this.toleranceDegrees = tol; }
    public void setMaxIntegral(double maxIntegral) { this.maxIntegral = maxIntegral; }
    public void setMaxTimeSeconds(double secs) { this.maxTimeSeconds = secs; }
    public void setPowerScaling(double p) { POWER = p; }

    // ---------------- polling (continuous sampling) ----------------
    /**
     * Start a background thread that continuously samples the calibrated angle and updates
     * the lastMeasuredAngle at roughly the given period in milliseconds.
     * Calling startPolling(0) will use the current pollPeriodMs.
     */
    public synchronized void startPolling(long periodMs) {
        if (analogEncoder == null) return;
        // stop any existing thread
        stopPolling();
        if (periodMs > 0) this.pollPeriodMs = periodMs;
        polling = true;
        pollThread = new Thread(() -> {
            while (polling) {
                try {
                    // sample and update lastMeasuredAngle via getCalibratedAngle()
                    getCalibratedAngle();
                    Thread.sleep(this.pollPeriodMs);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception ignored) {
                }
            }
        }, "SpindexerPollThread");
        pollThread.setDaemon(true);
        pollThread.start();
    }

    /** Stop background polling thread if running. */
    public synchronized void stopPolling() {
        polling = false;
        if (pollThread != null) {
            pollThread.interrupt();
            try { pollThread.join(50); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
            pollThread = null;
        }
    }

    public boolean isPolling() { return polling; }

    // ---------------- small helpers ----------------
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
