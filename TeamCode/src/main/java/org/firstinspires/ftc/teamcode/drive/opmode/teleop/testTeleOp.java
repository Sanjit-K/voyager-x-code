package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class testTeleOp extends OpMode {
    private Follower follower;
    private static final Pose startingPose = new Pose(0, 0, Math.toRadians(180));
    private BarIntake barIntake;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private ElapsedTime loopTimer;
    private ElapsedTime outtakeTimer;
    private LynxModule expansionHub;
    private static final double OFFSET = Math.toRadians(180.0);

    // Outtake routine state
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;
    private static final double OUTTAKE_DELAY_MS = 30;

    // RPM control
    private static final double RPM_STEP = 50.0;

    @Override
    public void init() {
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor");
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false, true);
        loopTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        follower.setStartingPose(startingPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Drive control
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET);

        // Intake control
        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        }

        // Spindex control
        if (gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }

        // RPM control
        if (gamepad1.dpadUpWasPressed()) {
            double newRPM = turret.getShooterRPM() + RPM_STEP;
            turret.setShooterRPM(newRPM);
            turret.on(); // Update velocity with new RPM
        } else if (gamepad1.dpadDownWasPressed()) {
            double newRPM = Math.max(0, turret.getShooterRPM() - RPM_STEP);
            turret.setShooterRPM(newRPM);
            if (newRPM > 0) {
                turret.on(); // Update velocity with new RPM
            } else {
                turret.off(); // Turn off if RPM reaches 0
            }
        }

        // Outtake routine trigger
        if (gamepad1.yWasPressed() && !outtakeInProgress) {
            startOuttakeRoutine();
        }

        // Handle outtake routine sequence
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }

        // Update spindexer
        spindexer.update();

        // Telemetry
        telemetry.addData("BarIntake Power", String.format(java.util.Locale.US, "%.3f", barIntake.getPower()));
        telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
        telemetry.addData("Turret RPM", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        telemetry.addData("Outtake Advance Count", outtakeAdvanceCount);
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();
        turret.on();

        // Step 2: Set kicker servo to kick
        kickerServo.kick();

        // Step 3: First advanceIntake call immediately
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call (30ms after last one)
        if (outtakeAdvanceCount < 3) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            // All 3 advanceIntake calls completed, set kicker back to normal
            kickerServo.normal();
            outtakeInProgress = false;
        }
    }
}
