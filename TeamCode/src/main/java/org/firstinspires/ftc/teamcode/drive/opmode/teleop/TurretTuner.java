package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretTuner extends OpMode {

    // Hardware config names - update to match your robot config
    private static final String SHOOTER_NAME = "shooter";
    private static final String TURRET_NAME = "turret";
    private static final String TURRET_ENCODER_NAME = "turretEncoder";
    private static final String TRANSFER_NAME = "transferMotor";

    private Turret turret;

    // tuning state
    private enum Param { KP, KI, KD, KSTATIC }
    private Param selected = Param.KP;

    private double stepKP = 0.001;
    private double stepKI = 0.0001;
    private double stepKD = 0.0001;
    private double stepKStatic = 0.01;

    private double targetAngle = 180;

    // button edge detection
    private boolean prevA = false, prevB = false, prevX = false, prevY = false;
    private boolean prevLB = false, prevRB = false, prevDUp = false, prevDDown = false;
    private boolean prevStart = false, prevBack = false;

    @Override
    public void init() {
        // Set booleans to false for typical wiring. Update booleans if your config needs reversing.
        turret = new Turret(hardwareMap,
                SHOOTER_NAME,
                TURRET_NAME,
                TURRET_ENCODER_NAME,
                TRANSFER_NAME,
                false, // shooterReversed
                false  // transferReversed
        );

        // Tracked-angle helpers were removed from Turret; start from a reasonable default.
        targetAngle = 180.0;
        turret.goToPosition(targetAngle);

        telemetry.addLine("Turret PID Tuner ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- handle selection buttons (A/B/X/Y) ---
        if (gamepad1.a && !prevA) selected = Param.KP;
        if (gamepad1.b && !prevB) selected = Param.KI;
        if (gamepad1.x && !prevX) selected = Param.KD;
        if (gamepad1.y && !prevY) selected = Param.KSTATIC;

        // --- change step scaling with D-pad up/down (adjust step magnitude) ---
        if (gamepad1.dpad_up && !prevDUp) {
            stepKP *= 10; stepKI *= 10; stepKD *= 10; stepKStatic *= 10;
        }
        if (gamepad1.dpad_down && !prevDDown) {
            stepKP = Math.max(stepKP / 10.0, 1e-9);
            stepKI = Math.max(stepKI / 10.0, 1e-12);
            stepKD = Math.max(stepKD / 10.0, 1e-12);
            stepKStatic = Math.max(stepKStatic / 10.0, 1e-6);
        }

        // --- adjust selected parameter with bumpers (RB increases, LB decreases) ---
        if (gamepad1.right_bumper && !prevRB) {
            modifySelected(+1.0);
        }
        if (gamepad1.left_bumper && !prevLB) {
            modifySelected(-1.0);
        }

        // --- fine adjustment using triggers (continuous while held) ---
        double triggerAdjust = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(triggerAdjust) > 0.05) {
            modifySelectedContinuous(triggerAdjust * 0.001); // small continuous steps
        }

        // --- target angle control: dpad left/right small/large steps ---
        if (gamepad1.dpad_left) targetAngle -= (gamepad1.right_bumper ? 10.0 : 1.0);
        if (gamepad1.dpad_right) targetAngle += (gamepad1.right_bumper ? 10.0 : 1.0);

        turret.goToPosition(targetAngle);

        // Telemetry
        telemetry.addData("Target Angle (cmd)", "%.2f", targetAngle);
        telemetry.addData("Voltage", "%.3f", turret.getTurretVoltage());
        telemetry.addLine("Controls: A=Kp B=Ki X=Kd Y=kStatic | RB/LB inc/dec | DPadUp/Down step *10 / /10");
        telemetry.update();

        // save button states for edge detection
        prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x; prevY = gamepad1.y;
        prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;
        prevDUp = gamepad1.dpad_up; prevDDown = gamepad1.dpad_down;
        prevStart = gamepad1.start; prevBack = gamepad1.back;
    }

    private void modifySelected(double direction) {
        switch (selected) {
            case KP:
                Turret.Kp = Math.max(0.0, Turret.Kp + direction * stepKP);
                break;
            case KI:
                Turret.Ki = Math.max(0.0, Turret.Ki + direction * stepKI);
                break;
            case KD:
                Turret.Kd = Math.max(0.0, Turret.Kd + direction * stepKD);
                break;
            case KSTATIC:
                Turret.kStatic = Math.max(0.0, Turret.kStatic + direction * stepKStatic);
                break;
        }
    }

    private void modifySelectedContinuous(double delta) {
        switch (selected) {
            case KP:
                Turret.Kp = Math.max(0.0, Turret.Kp + delta * stepKP * 10.0);
                break;
            case KI:
                Turret.Ki = Math.max(0.0, Turret.Ki + delta * stepKI * 10.0);
                break;
            case KD:
                Turret.Kd = Math.max(0.0, Turret.Kd + delta * stepKD * 10.0);
                break;
            case KSTATIC:
                Turret.kStatic = Math.max(0.0, Turret.kStatic + delta * stepKStatic * 10.0);
                break;
        }
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }
}