package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Spindexer {
    private final DcMotorEx spindexerMotor;
    private AnalogInput analogEncoder;
    private DigitalChannel distanceSensor;

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

    // Settings
    private static final double ANALOG_MAX_VOLTAGE = 3.3;

    // Calibration
    private double angleOffsetDegrees = 59.0;

    // Tracking
    private char[] filled = {'_', '_', '_'};
    private int intakeIndex = 0;

    // Positions (Degrees)
    // Intake: 0, 120, 240
    private final double[] INTAKE_ANGLES = {0.0, 120.0, 240.0};
    // Shoot: 180 (0.5), 300 (0.833), 60 (0.167)
    private final double[] SHOOT_ANGLES = {180.0, 300.0, 60.0};

    public Spindexer(HardwareMap hardwareMap, String motorName, String analogName, String distanceSensorName) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.analogEncoder = hardwareMap.get(AnalogInput.class, analogName);
        this.distanceSensor = hardwareMap.get(DigitalChannel.class, distanceSensorName);
        this.distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // --- Input Processing ---

    public double getAngleFromAnalog() {
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
    }

    public boolean update() {
        // Ball detection logic
        if (distanceSensor.getState()) {
            double currentAngle = getCalibratedAngle();
            for (int i = 0; i < 3; i++) {
                if (Math.abs(smallestAngleDifference(currentAngle, INTAKE_ANGLES[i])) < 10.0) {
                    // Ball detected at slot i
                    if (filled[i] == '_') {
                        filled[i] = 'X'; // Mark as filled (unknown color)
                        // Auto-advance if not full
                        if (!isFull()) {
                            advanceIntake();
                        }
                    }
                }
            }
        }


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
        fTerm = Math.signum(error) * kStatic;

        double out = pTerm + iTerm + dTerm + fTerm;

        // Clamp
        if (out > 1.0) out = 1.0;
        if (out < -1.0) out = -1.0;

        spindexerMotor.setPower(out);

        return true;
    }

    public void stop() {
        spindexerMotor.setPower(0.0);
    }


    // --- Tracking & Positions ---

    public void setIntakeIndex(int index) {
        intakeIndex = index % 3;
        if (intakeIndex < 0) intakeIndex += 3;
        startMoveToAngle(INTAKE_ANGLES[intakeIndex]);
    }

    public void advanceIntake() {
        intakeIndex = (intakeIndex + 1) % 3;
        setIntakeIndex(intakeIndex);
    }

    public void retreatIntake() {
        intakeIndex = (intakeIndex + 2) % 3; // equivalent to -1
        setIntakeIndex(intakeIndex);
    }

    public void setShootIndex(int index) {
        index %= 3;
        if (index < 0) index += 3;
        startMoveToAngle(SHOOT_ANGLES[index]);
    }

    public void setColorAtPos(char color, int index) {
        if (index >= 0 && index < 3) filled[index] = color;
    }

    public void setColorAtPos(char color) {
        setColorAtPos(color, intakeIndex);
    }

    public boolean isFull() {
        for (char c : filled) {
            if (c == '_') return false;
        }
        return true;
    }

    public void clearTracking() {
        filled[0] = '_';
        filled[1] = '_';
        filled[2] = '_';
    }

    public void moveToZero() {
        startMoveToAngle(0.0);
        while (update()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public int getIntakeIndex() {
        return intakeIndex;
    }

    public double getPosition() {
        return getCalibratedAngle();
    }

    public char[] getFilled() {
        return filled;
    }

    // --- Helpers ---
    private double normalizeAngleDegrees(double a) {
        double res = a % 360.0;
        if (res < 0) res += 360.0;
        return res;
    }

    private double smallestAngleDifference(double target, double current) {
        return Math.IEEEremainder(target - current, 360.0);
    }
}
