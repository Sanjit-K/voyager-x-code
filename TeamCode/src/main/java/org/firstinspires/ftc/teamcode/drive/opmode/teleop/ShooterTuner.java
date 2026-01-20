/*
 * ShooterVelocityTuner.java
 *
 * Tunes the built-in REV Hub velocity PIDF for a shooter motor (RUN_USING_ENCODER + setVelocity()).
 *
 * Hardware:
 *   - Motor named "shooter" (change SHOOTER_NAME if needed)
 *
 * Controls (gamepad1):
 *   - A: toggle shooter ON/OFF
 *   - Dpad Up/Down: targetRPM += / -= rpmStep
 *   - Dpad Left/Right: rpmStep /=2 or *=2
 *   - Y: cycle which coefficient you edit (P -> I -> D -> F)
 *   - Left stick Y: adjust selected coefficient (up = increase)
 *   - LB: fine adjust, RB: coarse adjust
 *   - B: toggle auto-step between rpmA and rpmB (good for watching response)
 *
 * Notes:
 *   - COUNTS_PER_REV must match the encoder used by the motor controller for getVelocity().
 *     In your Turret code you used 28, so this uses 28 too.
 *   - This tunes the motor controller’s internal loop, not a custom software PID.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Velocity Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";

    // Must match your RPM conversion
    private static final double COUNTS_PER_REV = 28.0;

    // Target control
    private double targetRPM = 2500.0;
    private double rpmStep = 50.0;

    // Auto-step (watch spin-up and disturbance recovery)
    private boolean autoStep = false;
    private double rpmA = 2000.0;
    private double rpmB = 3000.0;
    private double autoPeriodS = 2.0;
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Velocity PIDF coefficients (REV Hub internal)
    private double kP, kI, kD, kF;

    private boolean shooterOn = false;

    private enum Param { P, I, D, F }
    private Param selected = Param.P;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // For flywheels, FLOAT usually feels better than BRAKE.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Load current coefficients from the controller (so you start from known values)
        PIDFCoefficients coeffs = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
        kF = coeffs.f;

        autoTimer.reset();

        waitForStart();

        boolean prevA = false, prevB = false, prevY = false;
        boolean prevLeft = false, prevRight = false;

        while (opModeIsActive()) {
            // --- Button edges ---
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;

            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (a && !prevA) shooterOn = !shooterOn;
            if (b && !prevB) { autoStep = !autoStep; autoTimer.reset(); }
            if (y && !prevY) selected = next(selected);

            if (left && !prevLeft) rpmStep = Math.max(1.0, rpmStep / 2.0);
            if (right && !prevRight) rpmStep = Math.min(2000.0, rpmStep * 2.0);

            // --- Target RPM control ---
            if (gamepad1.dpad_up) targetRPM += rpmStep;
            if (gamepad1.dpad_down) targetRPM -= rpmStep;
            targetRPM = Math.max(0.0, targetRPM);

            // Auto-step target
            if (autoStep && autoTimer.seconds() >= autoPeriodS) {
                autoTimer.reset();
                targetRPM = (Math.abs(targetRPM - rpmA) < 1e-6) ? rpmB : rpmA;
            }

            // --- Live coefficient editing ---
            double stick = -gamepad1.left_stick_y; // up = +, down = -
            if (Math.abs(stick) > 0.08) {
                double scale = 1.0;
                if (gamepad1.left_bumper) scale = 0.2;  // fine
                if (gamepad1.right_bumper) scale = 5.0; // coarse

                // Step sizes per loop (simple, stable enough for tuning)
                switch (selected) {
                    case P:
                        kP = clampNonNeg(kP + stick * 1.0 * scale);
                        break;
                    case I:
                        kI = clampNonNeg(kI + stick * 0.5 * scale);
                        break;
                    case D:
                        kD = clampNonNeg(kD + stick * 1.0 * scale);
                        break;
                    case F:
                        kF = clampNonNeg(kF + stick * 1.0 * scale);
                        break;
                }
            }

            // Apply coefficients (safe to call often)
            shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            // --- Command motor velocity ---
            if (shooterOn) {
                double targetTPS = targetRPM * COUNTS_PER_REV / 60.0;
                shooter.setVelocity(targetTPS);
            } else {
                shooter.setPower(0.0);
            }

            // --- Telemetry ---
            double measuredTPS = shooter.getVelocity();
            double measuredRPM = measuredTPS * 60.0 / COUNTS_PER_REV;
            double errRPM = targetRPM - measuredRPM;

            telemetry.addLine("=== Shooter Velocity Tuner ===");
            telemetry.addData("Shooter tuning", shooterOn ? "ON (A)" : "OFF (A)");
            telemetry.addData("Auto-step (B)", autoStep);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("RPM step (Dpad L/R)", "%.1f", rpmStep);

            telemetry.addLine();
            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            telemetry.addData("Error RPM", "%.1f", errRPM);

            telemetry.addLine();
            telemetry.addData("Selected (Y)", selected);
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.4f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("kF", "%.4f", kF);

            telemetry.addLine();
            telemetry.addData("Velocity (ticks/s)", "%.1f", measuredTPS);

            telemetry.update();

            prevA = a; prevB = b; prevY = y;
            prevLeft = left; prevRight = right;
        }

        shooter.setPower(0.0);
    }

    private static Param next(Param p) {
        switch (p) {
            case P: return Param.I;
            case I: return Param.D;
            case D: return Param.F;
            default: return Param.P;
        }
    }

    private static double clampNonNeg(double v) {
        return Math.max(0.0, v);
    }
}
