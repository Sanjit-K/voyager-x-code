package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;

/**
 * Workflow requested:
 * 1) Turn turret to SCAN_TURRET_DEG and prep AprilTag scanning
 * 2) Follow ShootPreset
 * 3) At end, scan AprilTag -> choose shooting angle -> shoot 3 balls
 * 4) Follow Pickup2
 * 5) Follow Shoot2
 * 6) Wait 200ms after Shoot2 completes -> shoot 2 balls
 * 7) Park at PARK_SPEED (default 0.7)
 *
 * Includes "motif classification logic" hooks (ball classification + keep/eject).
 */
@Autonomous(name = "Blue 9 Ball Auto", group = "Autonomous")
@Configurable
public class BlueNineBallAuto extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems --------------------
    private BarIntake barIntake;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;

    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 308;     // turret angle while scanning for tag
    public static double DEFAULT_SHOOT_TURRET_DEG = 286; // fallback angle if tag not seen
    public static double SHOOT_RPM = 2750;

    public static double PARK_SPEED = 0.70;         // follower speed scalar for park
    public static int WAIT_AFTER_SHOOT2_MS = 200;   // requested settle wait

    // Outtake cadence
    public static double OUTTAKE_DELAY_MS = 700;

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
        }
        pathState = s;
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private int outtakeTargetShots = 3;        // set to 3 for first volley, 2 for second
    private double lastAdvanceTimeMs = 0.0;

    // -------------------- AprilTag scanning (HOOK / stub) --------------------
    // Replace this with your actual AprilTag pipeline wrapper.
    private final TagScanner tagScanner = new TagScanner();

    // -------------------- Motif (ball) classification --------------------
    // You asked for "ball (motif) classification logic". Since I don’t have your exact API,
    // I’m implementing it as a clear pattern you can wire into your Spindexer/ColorSensor methods.
    private enum BallClass { ALLIANCE_OK, OPPONENT, UNKNOWN }
    public static boolean EJECT_OPPONENT_BALLS = true;

    // -----------------------------------------------------------------------------------------
    // init / start / loop
    // -----------------------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Starting pose: you set (72, 8, 90deg) in your sample.
        // IMPORTANT: this must match the *start* of the first path you actually follow.
        // Your first path ShootPreset starts at (33,136) in the sample Paths, which does NOT match.
        // You should either:
        //  - change this starting pose to (33,136,180deg), OR
        //  - rebuild ShootPreset to start at (72,8,90deg).
        //
        // I’m keeping your provided start pose but marking it as critical.
        follower.setStartingPose(new Pose(22.55, 123.14, Math.toRadians(180)));

        // Subsystems
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(
                hardwareMap,
                "spindexerMotor",
                "spindexerAnalog",
                "distanceSensor",
                colorSensor
        );
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(
                hardwareMap,
                "shooter",
                "turret",
                "turretEncoder",
                "transferMotor",
                false,
                true,
                true,
                308
        );

        // Paths
        paths = new Paths(follower);

        // Startup config
        kickerServo.normal();
        barIntake.spinIntake();
        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        // Prep tag scanner
        tagScanner.init(hardwareMap);

        // Put turret into scanning pose immediately
        turret.goToPosition(SCAN_TURRET_DEG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        setState(0);

        stateTimer.reset();

        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        // Begin scanning immediately
        tagScanner.start();
    }

    @Override
    public void loop() {
        // 1) Always update follower first
        follower.update();

        // 2) Always keep shooter ready
        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        // turret angle is set by state machine (scan angle vs shoot angle)
        // turret.goToPosition(...) is called in autonomousUpdate()

        // 3) Update spindexer and run motif classification
        spindexer.update();
        handleMotifClassification();

        // 4) Run state machine
        autonomousUpdate();

        // 5) Telemetry
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", safeGetBalls());
        panelsTelemetry.debug("Tag Seen", tagScanner.hasTag());
        panelsTelemetry.debug("Tag Id", tagScanner.getTagId());
        panelsTelemetry.update(telemetry);
    }

    // -----------------------------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------------------------

    private void autonomousUpdate() {
        // Outtake blocks transitions
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {

            // ------------------------------------------------------------
            // 0) Turret to scan angle, start path ShootPreset
            // ------------------------------------------------------------
            case 0:
                turret.goToPosition(SCAN_TURRET_DEG);
                tagScanner.update(); // keep pipeline warm

                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            // ------------------------------------------------------------
            // 1) When ShootPreset finishes: scan tag -> choose shooting angle -> shoot 3
            // ------------------------------------------------------------
            case 1:
                // keep scanning while driving / settling
                tagScanner.update();
                turret.goToPosition(SCAN_TURRET_DEG);

                if (!follower.isBusy()) {
                    // Freeze a tag decision here (read once at the shoot location)
                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    // (Optional) small aim settle; you can remove if not needed
                    if (stateTimer.milliseconds() < 150) return;

                    // Shoot 3 balls
                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(2);
                }
                break;

            // ------------------------------------------------------------
            // 2) After volley #1: follow Pickup2
            // ------------------------------------------------------------
            case 2:
                if (!outtakeInProgress) {
                    barIntake.spinIntake();
                    follower.followPath(paths.Pickup2);
                    setState(3);
                }
                break;

            // ------------------------------------------------------------
            // 3) When Pickup2 finishes: go Shoot2
            // ------------------------------------------------------------
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    setState(4);
                }
                break;

            // ------------------------------------------------------------
            // 4) When Shoot2 finishes: wait 200ms -> shoot 2 balls
            // ------------------------------------------------------------
            case 4:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOOT2_MS) return;

                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    // Shoot 2 balls
                    outtakeTargetShots = 2;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(5);
                }
                break;

            // ------------------------------------------------------------
            // 5) After volley #2: Park at PARK_SPEED (0.7)
            // ------------------------------------------------------------
            case 5:
                if (!outtakeInProgress) {
                    // If your follower supports speed scalar overload:
                    // follower.followPath(paths.Park, PARK_SPEED, false);
                    // If it does NOT, replace with follower.followPath(paths.Park);
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(6);
                }
                break;

            // ------------------------------------------------------------
            // 6) Done once Park completes
            // ------------------------------------------------------------
            case 6:
                if (!follower.isBusy()) {
                    setState(7);
                }
                break;

            case 7:
                // End state
                break;
        }
    }

    // -----------------------------------------------------------------------------------------
    // Outtake (kick + advance N shots with delay)
    // -----------------------------------------------------------------------------------------

    private void startOuttakeRoutine(int targetShots) {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTargetShots = targetShots;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        turret.on();
        turret.transferOn();

        kickerServo.kick();

        // Advance immediately for shot #1
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTimeMs = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double now = outtakeTimer.milliseconds();

        // Advance for shot #2..#N
        if (outtakeAdvanceCount < outtakeTargetShots) {
            if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTimeMs = now;
            }
            return;
        }

        // After final advance, wait one more delay then finish
        if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
            kickerServo.normal();
            spindexer.clearTracking();
            barIntake.spinIntake();
            outtakeInProgress = false;
        }
    }

    // -----------------------------------------------------------------------------------------
    // Motif (ball) classification logic
    // -----------------------------------------------------------------------------------------

    /**
     * Pattern:
     * - spindexer.update() keeps internal state current
     * - classify the “current/next” ball using ColorSensor (or spindexer’s helpers)
     * - if it’s opponent ball, optionally eject/reject it
     *
     * Because I don’t have your exact APIs (e.g., spindexer.hasNewBall(), spindexer.reject(), etc),
     * I’m writing this in a way that’s very easy to hook to whatever methods you do have.
     */
    private void handleMotifClassification() {
        // If you only want to classify while intaking (not while outtaking)
        if (outtakeInProgress) return;

        // If your spindexer has no balls or no “new ball” event, you can skip.
        // Replace these guards with your real conditions.
        int balls = safeGetBalls();
        if (balls <= 0) return;

        BallClass cls = classifyBall();

        if (EJECT_OPPONENT_BALLS && cls == BallClass.OPPONENT) {
            // ---- IMPORTANT HOOK: implement your actual reject/eject method here ----
            // Examples (pick the one your codebase actually has):
            // spindexer.reject();          // if you have a dedicated reject action
            // spindexer.reverseBriefly();  // if you reverse the spindexer motor
            // barIntake.reverse();         // if intake can spit out
            //
            // For now, I’ll do a conservative action: stop intake to avoid feeding it deeper.
            barIntake.stop();
        } else if (cls == BallClass.ALLIANCE_OK) {
            // keep intaking
            barIntake.spinIntake();
        }
    }

    /**
     * Replace the body with your real “motif” classification:
     * - likely based on colorSensor readings (HSV, RGB thresholds)
     * - or spindexer’s own classification output if it already does it
     */
    private BallClass classifyBall() {
        // ---- IMPORTANT HOOK: You must map this to your own ColorSensor API ----
        // Some teams implement:
        //   colorSensor.isBlue(), colorSensor.isRed(), colorSensor.confidence(), etc.
        //
        // If your ColorSensor has methods, use them here.
        //
        // Placeholder: assume unknown.
        return BallClass.UNKNOWN;
    }

    // -----------------------------------------------------------------------------------------
    // AprilTag -> shoot angle selection
    // -----------------------------------------------------------------------------------------

    private double chooseShootAngleFromTagOrDefault() {
        // Keep pipeline fresh once
        tagScanner.update();

        if (!tagScanner.hasTag()) return DEFAULT_SHOOT_TURRET_DEG;

        int id = tagScanner.getTagId();

        // Example mapping (EDIT THESE):
        // Tag 1 -> angle A, Tag 2 -> angle B, Tag 3 -> angle C
        // Keep it simple and deterministic.
        if (id == 1) return 286;
        if (id == 2) return 290;
        if (id == 3) return 294;

        return DEFAULT_SHOOT_TURRET_DEG;
    }

    // -----------------------------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------------------------

    private int safeGetBalls() {
        try {
            return spindexer.getBalls();
        } catch (Throwable t) {
            return -1; // if your Spindexer doesn’t have getBalls()
        }
    }

    // -----------------------------------------------------------------------------------------
    // Paths (your provided geometry)
    // -----------------------------------------------------------------------------------------

    public static class Paths {
        public PathChain ShootPreset;
        public PathChain Pickup1;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.55, 123.14),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(43.359, 80.552),
                                    new Pose(15.381, 83.807)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.381, 83.807),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(54.022, 55.831),
                                    new Pose(14.254, 59.254)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.254, 59.254),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),
                                    new Pose(38.564, 33.525)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    // -----------------------------------------------------------------------------------------
    // TagScanner STUB (replace with your actual VisionPortal / AprilTagProcessor wrapper)
    // -----------------------------------------------------------------------------------------

    private static class TagScanner {
        private boolean started = false;
        private boolean hasTag = false;
        private int tagId = -1;

        public void init(com.qualcomm.robotcore.hardware.HardwareMap hw) {
            // TODO: init your camera + AprilTag pipeline here
        }

        public void start() {
            started = true;
        }

        public void update() {
            if (!started) return;

            // TODO: poll AprilTag detections here and update hasTag/tagId
            // Example:
            //   List<AprilTagDetection> dets = aprilTag.getDetections();
            //   if (!dets.isEmpty()) { hasTag = true; tagId = dets.get(0).id; }

            // Placeholder: keep whatever last values were
        }

        public boolean hasTag() { return hasTag; }
        public int getTagId() { return tagId; }
    }
}
