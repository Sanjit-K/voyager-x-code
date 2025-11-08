package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.ColorSensor;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.RobotHeading;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;

@Configurable
@TeleOp
public class BlueTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(48, 72, Math.toRadians(180+122));
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private ColorSensor colorSensor;

    private double turretPower = 0.60;
    private boolean turretSpinning = false;         // shooters running?
    private boolean shootersLockedOn = false;       // set true after touchpad press (“never off”)

    // Intake/turret helpers and state
    private YawServo yawServo;
    private TurretConstants turretConstants;
    private IntakeServos intakeServos;
    private LaunchServos launchServos;
    private LaunchMotors launchMotors;
    private RobotHeading robotHeading;

    private boolean frontSpinningBar = false;
    private boolean frontSpinningWheels = false;
    private boolean backSpinningBar = false;
    private boolean backSpinningWheels = false;
    private boolean launchServosActive = false;
    private boolean yawServoDirection = false;

    // “swap which intake is active” state for G2 ▢ (square)
    private boolean frontIntakeSelected = true;

    // Edge-detect for analog triggers on G2
    private boolean g2LtPressedPrev = false;
    private boolean g2RtPressedPrev = false;

    // Cooldown so the G2 right-stick “power up/down” doesn’t spam
    private final ElapsedTime powerNudgeTimer = new ElapsedTime();
    private static final double POWER_NUDGE_COOLDOWN_S = 0.25;
    private static final double POWER_NUDGE = 0.05;
    private static final double STICK_ACTIVATE = 0.7;

    // Alliance POV offset: 180 = Blue, 0 = Red
    private final double offset = Math.toRadians(180.0);

    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) enableAction.run(); else disableAction.run();
        return !currentState;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.setStartingPose(startingPose);

        intakeServos = new IntakeServos(hardwareMap,
                "leftForward", "barFront", "rightForward",
                "leftBack",   "barBack",  "rightBack");

        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR", "servoTop");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");

        turretConstants = new TurretConstants();
        try {
            yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        } catch (Exception e) {
            yawServo = null;
        }
        robotHeading = new RobotHeading(follower, 0, 144);
        if (yawServo != null) yawServo.back();

        // Color sensor is optional
        try {
            colorSensor = new ColorSensor(hardwareMap, "color");
            telemetryM.debug("Color Sensor", "initialized");
        } catch (Exception e) {
            colorSensor = null;
            telemetryM.debug("Color Sensor", "NOT FOUND: check config name 'color'");
        }

        powerNudgeTimer.reset();
        colorSensor.setDelayMillis(250);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Keep turret model updated
        turretConstants.setRobotPose(follower.getPose().getX(), follower.getPose().getY());
        turretConstants.update();

        // Early exit back to manual when an automated routine finishes
        if (automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // =====================  GAMEPAD 1 (DRIVE)  =====================
        // DRIVE LOCK (hold): G1 Left Trigger — lock robot in place while held
        boolean g1Locked = gamepad1.left_trigger > 0.5;

        if (!automatedDrive) {
            // Aim-at-goal trigger MOVED: Share → G1 Left Stick Button
            if (gamepad1.leftStickButtonWasPressed()) {
                if (robotHeading != null) {
                    robotHeading.aimAtGoal(true);
                    automatedDrive = true;
                }
            }

            if (gamepad1.rightStickButtonWasPressed()){
                if (robotHeading != null) {
                    robotHeading.aimAtGoal(false);
                    automatedDrive = true;
                }
            }

            // Field-centric drive with alliance offset, suppressed if locked
            if (g1Locked) {
                follower.setTeleOpDrive(0, 0, 0, false, offset);
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false,
                        offset
                );
            }
        }

        // =====================  GAMEPAD 2 (ACTIONS)  =====================

        // ▢ (Square) — toggle which intake side is ACTIVE (front <-> back)
        if (gamepad2.xWasPressed()) { // PS4 Square is X in FTC mapping
            frontIntakeSelected = !frontIntakeSelected;
            if (frontIntakeSelected) {
                // enable front wheels, disable back wheels (respect slow-creep logic later)
                frontSpinningWheels = true;
                backSpinningWheels  = false;
                intakeServos.enableFrontWheels();
                intakeServos.disableBackWheels();
            } else {
                frontSpinningWheels = false;
                backSpinningWheels  = true;
                intakeServos.disableFrontWheels();
                intakeServos.enableBackWheels();
            }
        }

        // 🔹 NEW: Reset heading to 0° (G2 D-pad LEFT)
        if (gamepad2.dpadLeftWasPressed()) {
            if (robotHeading != null) {
                robotHeading.resetHeading(); // sets IMU / follower heading to 0 radians
                telemetryM.debug("Heading Reset", "Robot heading set to 0°");
            }
        }

        // L1 — FRONT BAR (moved from G1 A)
        if (gamepad2.leftBumperWasPressed()) {
            frontSpinningBar = toggle(frontSpinningBar, intakeServos::enableFrontBar, intakeServos::disableFrontBar);
        }

        // L2 — FRONT WHEELS toggle (moved from G1 L1)
        boolean g2LtPressed = gamepad2.left_trigger > 0.5;
        if (g2LtPressed && !g2LtPressedPrev) {
            frontSpinningWheels = toggle(frontSpinningWheels, intakeServos::enableFrontWheels, intakeServos::disableFrontWheels);
        }
        g2LtPressedPrev = g2LtPressed;

        // R2 — BACK WHEELS toggle (moved from G1 R1/L2 note)
        boolean g2RtPressed = gamepad2.right_trigger > 0.5;
        if (g2RtPressed && !g2RtPressedPrev) {
            backSpinningWheels = toggle(backSpinningWheels, intakeServos::enableBackWheels, intakeServos::disableBackWheels);
        }
        g2RtPressedPrev = g2RtPressed;

        // (unchanged) G2 A/B slow creep while held — restores prior toggled states when released
        boolean frontSlowHeld = gamepad2.a;
        boolean backSlowHeld  = gamepad2.b;
        if (frontSlowHeld) {
            intakeServos.enableFrontWheelsSlow();
        } else {
            if (frontSpinningWheels) intakeServos.enableFrontWheels(); else intakeServos.disableFrontWheels();
        }
        if (backSlowHeld) {
            intakeServos.enableBackWheelsSlow();
        } else {
            if (backSpinningWheels) intakeServos.enableBackWheels(); else intakeServos.disableBackWheels();
        }

        // (moved) BACK BAR — from G1 B to G2 L2 per your note “L2 - backbar”; to avoid conflict with wheels on L2, map to G2 R1 (right bumper)
        if (gamepad2.rightBumperWasPressed()) {
            backSpinningBar = toggle(backSpinningBar, intakeServos::enableBackBar, intakeServos::disableBackBar);
        }

        // Yaw servo front/back — stays on G2 X (triangle/square already used)
        if (gamepad2.yWasPressed()) { // PS4 Triangle is Y in FTC mapping
            yawServoDirection = toggle(yawServoDirection,
                    () -> { if (yawServo != null) yawServo.front(); },
                    () -> { if (yawServo != null) yawServo.back();  });
        }
        if (gamepad2.dpadRightWasPressed()) { if (yawServo != null) yawServo.center(); }

        // Launch servos TOGGLE — moved from G1 L3 to G2 L3
        if (gamepad2.leftStickButtonWasPressed()) {
            launchServosActive = toggle(launchServosActive, launchServos::enable, launchServos::disable);
        }

        // TOUCHPAD — “Turn on shooters (never turn off)” (moved from G1 OPTIONS)
        // Some wrappers expose touchpad; as a fallback, also honor G2 OPTIONS.
        boolean touchpadPressed = false;
        try {
            touchpadPressed = gamepad2.touchpadWasPressed();
        } catch (Throwable ignored) {}
        if (touchpadPressed || gamepad2.optionsWasPressed()) {
            shootersLockedOn = true;
            turretSpinning   = true;
            launchMotors.set(turretPower);
        }

        // G2 Right stick up/down adjusts turret power (up = increase, down = decrease)
        if (powerNudgeTimer.seconds() > POWER_NUDGE_COOLDOWN_S) {
            if (gamepad2.right_stick_y < -STICK_ACTIVATE) { // stick up
                turretPower = Math.min(1.0, turretPower + POWER_NUDGE);
                powerNudgeTimer.reset();
            } else if (gamepad2.right_stick_y > STICK_ACTIVATE) { // stick down
                turretPower = Math.max(0.0, turretPower - POWER_NUDGE);
                powerNudgeTimer.reset();
            }
        }
        if (turretSpinning) launchMotors.set(turretPower);

        // NOTE: we no longer allow turning shooters off via a button if shootersLockedOn == true
        // If you still want a safety kill, add one explicitly (e-stop).

        // =====================  TARGET + DETECTION LOGIC  =====================
        // Use detection in place of purple/green logic (left trigger on G1 is now drive lock; keep detection on G1 LT threshold for consistency if desired)
        boolean detected = false;
        int brightness = 0;
        if (colorSensor != null) {
            brightness = colorSensor.getBrightness8bit();
            detected = colorSensor.delayedDetection();
        }
        if (detected && gamepad1.left_trigger < 0.5){
            launchServos.disable();
            intakeServos.disableFrontWheels();
            intakeServos.disableBackWheels();
            frontSpinningWheels = false;
            backSpinningWheels = false;
            launchServosActive = false;
        } else if (gamepad1.left_trigger >= 0.5){
            launchServos.enable();
        }

        // =====================  TELEMETRY  =====================
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Drive Locked (G1 LT)", g1Locked);
        telemetryM.debug("Turret Power", turretPower);
        telemetryM.debug("Shooters Spinning", turretSpinning);
        telemetryM.debug("Shooters Locked On", shootersLockedOn);
        telemetryM.debug("Goal Bearing (deg)", turretConstants.getBearingDeg());
        telemetryM.debug("Distance to Goal", turretConstants.getDistance());
        telemetryM.debug("Target RPM (est)", turretConstants.getTargetRPM());
        telemetryM.debug("Yaw Servo Pos", yawServo != null ? yawServo.getPosition() : "N/A");
        telemetryM.debug("Brightness(0-255)", brightness);
        telemetryM.debug("Detected", detected);
        telemetryM.debug("Active Intake", frontIntakeSelected ? "FRONT" : "BACK");
        telemetryM.update(telemetry);
    }
}