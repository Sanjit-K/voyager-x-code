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
 * Sequence requested (same methodology):
 *  ShootPreset -> Pickup1 -> Shoot1 -> Pickup2 -> Shoot2 -> Pickup3 -> Shoot3 -> Park
 *
 * Methodology preserved:
 *  - turret to SCAN angle before/while driving to first shoot
 *  - AprilTag scan at shoot point -> choose shooting angle -> volley
 *  - non-blocking outtake routine (kick + spindexer advances with delay)
 *  - motif (ball) classification logic hook
 *  - optional settle wait after arriving at shoot points (WAIT_AFTER_SHOT_MS)
 *  - park uses speed scalar PARK_SPEED (default 0.7)
 */
@Autonomous(name = "Blue 12 Ball Auto", group = "Autonomous")
@Configurable
public class BlueTwelveBallAuto extends OpMode {

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

    // -------------------- Config --------------------
    public static double SCAN_TURRET_DEG = 308;
    public static double DEFAULT_SHOOT_TURRET_DEG = 286;
    public static double SHOOT_RPM = 3250;

    public static double OUTTAKE_DELAY_MS = 700;
    public static int WAIT_AFTER_SHOT_MS = 200;     // settle wait after reaching each shoot pose
    public static double PARK_SPEED = 0.70;

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
    private int outtakeTargetShots = 3; // always 3 in this auto (shoot preset + shoot1 + shoot2 + shoot3)
    private double lastAdvanceTimeMs = 0.0;

    // -------------------- AprilTag scanning (HOOK / stub) --------------------
    private final TagScanner tagScanner = new TagScanner();

    // -------------------- Motif (ball) classification --------------------
    private enum BallClass { ALLIANCE_OK, OPPONENT, UNKNOWN }
    public static boolean EJECT_OPPONENT_BALLS = true;

    // -----------------------------------------------------------------------------------------
    // init / start / loop
    // -----------------------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // IMPORTANT: must match first path start (ShootPreset starts at 33,136,180deg in Paths below)
        follower.setStartingPose(new Pose(33, 136, Math.toRadians(180)));

        // Subsystems (names match your other autos)
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

        // Build paths
        paths = new Paths(follower);

        // Startup
        kickerServo.normal();
        barIntake.spinIntake();

        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        // Vision
        tagScanner.init(hardwareMap);

        // Pre-aim turret to scan angle
        turret.goToPosition(SCAN_TURRET_DEG);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        setState(0);

        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        barIntake.spinIntake();

        tagScanner.start();
    }

    @Override
    public void loop() {
        // Update follower first
        follower.update();

        // Keep shooter ready
        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);

        // Update spindexer + classification
        spindexer.update();
        handleMotifClassification();

        // Run state machine
        autonomousUpdate();

        // Telemetry
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
            // 0) Pre-aim turret for tag scan, drive to ShootPreset
            // ------------------------------------------------------------
            case 0:
                turret.goToPosition(SCAN_TURRET_DEG);
                tagScanner.update();

                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            // ------------------------------------------------------------
            // 1) ShootPreset complete -> scan tag -> shoot 3 -> go Pickup1
            // ------------------------------------------------------------
            case 1:
                tagScanner.update();
                turret.goToPosition(SCAN_TURRET_DEG);

                if (!follower.isBusy()) {
                    // choose shooting angle at the shooting location
                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOT_MS) return;

                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(2);
                }
                break;

            // 2) After volley -> Pickup1
            case 2:
                if (!outtakeInProgress) {
                    barIntake.spinIntake();
                    follower.followPath(paths.Pickup1);
                    setState(3);
                }
                break;

            // 3) Pickup1 complete -> Shoot1
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot1);
                    setState(4);
                }
                break;

            // 4) Shoot1 complete -> wait -> shoot 3 -> Pickup2
            case 4:
                tagScanner.update();
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOT_MS) return;

                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(5);
                }
                break;

            // 5) After volley -> Pickup2
            case 5:
                if (!outtakeInProgress) {
                    barIntake.spinIntake();
                    follower.followPath(paths.Pickup2);
                    setState(6);
                }
                break;

            // 6) Pickup2 complete -> Shoot2
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    setState(7);
                }
                break;

            // 7) Shoot2 complete -> wait -> shoot 3 -> Pickup3
            case 7:
                tagScanner.update();
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOT_MS) return;

                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(8);
                }
                break;

            // 8) After volley -> Pickup3
            case 8:
                if (!outtakeInProgress) {
                    barIntake.spinIntake();
                    follower.followPath(paths.Pickup3);
                    setState(9);
                }
                break;

            // 9) Pickup3 complete -> Shoot3
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot3);
                    setState(10);
                }
                break;

            // 10) Shoot3 complete -> wait -> shoot 3 -> Park
            case 10:
                tagScanner.update();
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOT_MS) return;

                    double shootAngle = chooseShootAngleFromTagOrDefault();
                    turret.goToPosition(shootAngle);

                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(11);
                }
                break;

            // 11) After volley -> Park (speed scalar)
            case 11:
                if (!outtakeInProgress) {
                    // If your follower doesn't support this overload, replace with follower.followPath(paths.Park);
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(12);
                }
                break;

            // 12) Done when park completes
            case 12:
                if (!follower.isBusy()) {
                    setState(13);
                }
                break;

            case 13:
                // End state
                break;
        }
    }

    // -----------------------------------------------------------------------------------------
    // Outtake routine (kick + advance N shots with delay)
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

        // First advance immediately
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTimeMs = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double now = outtakeTimer.milliseconds();

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
    // Motif (ball) classification logic (HOOK)
    // -----------------------------------------------------------------------------------------

    private void handleMotifClassification() {
        if (outtakeInProgress) return;

        // Only attempt classification if you actually have something to classify
        int balls = safeGetBalls();
        if (balls <= 0) return;

        BallClass cls = classifyBall();

        if (EJECT_OPPONENT_BALLS && cls == BallClass.OPPONENT) {
            // HOOK: replace with your real reject action (reverse spindexer/intake, open gate, etc.)
            barIntake.stop();
        } else if (cls == BallClass.ALLIANCE_OK) {
            barIntake.spinIntake();
        }
    }

    private BallClass classifyBall() {
        // HOOK: map to your real ColorSensor/Spindexer classification
        return BallClass.UNKNOWN;
    }

    // -----------------------------------------------------------------------------------------
    // AprilTag -> shoot angle (HOOK)
    // -----------------------------------------------------------------------------------------

    private double chooseShootAngleFromTagOrDefault() {
        tagScanner.update();
        if (!tagScanner.hasTag()) return DEFAULT_SHOOT_TURRET_DEG;

        int id = tagScanner.getTagId();
        if (id == 1) return 286;
        if (id == 2) return 290;
        if (id == 3) return 294;

        return DEFAULT_SHOOT_TURRET_DEG;
    }

    private int safeGetBalls() {
        try {
            return spindexer.getBalls();
        } catch (Throwable t) {
            return -1;
        }
    }

    // -----------------------------------------------------------------------------------------
    // Paths (ShootPreset -> Pickup1 -> Shoot1 -> Pickup2 -> Shoot2 -> Pickup3 -> Shoot3 -> Park)
    // -----------------------------------------------------------------------------------------

    public static class Paths {
        public PathChain ShootPreset;
        public PathChain Pickup1;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;

        // NEW: pickup3 + shoot3 added for your extended methodology
        public PathChain Pickup3;
        public PathChain Shoot3;

        public PathChain Park;

        public Paths(Follower follower) {

            // These match the pathing you showed
            ShootPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(322.55, 123.14),
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

            // ------------------------------------------------------------
            // You MUST replace these two with your actual visualizer-exported points.
            // I’m putting safe placeholders that continue from Shoot2 end (48,96).
            // ------------------------------------------------------------

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(55.000, 40.000),
                                    new Pose(10.000, 40.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 40.000),
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
    // TagScanner stub (replace with your actual pipeline wrapper)
    // -----------------------------------------------------------------------------------------

    private static class TagScanner {
        private boolean started = false;
        private boolean hasTag = false;
        private int tagId = -1;

        public void init(com.qualcomm.robotcore.hardware.HardwareMap hw) {
            // TODO: init camera + AprilTag processor
        }

        public void start() { started = true; }

        public void update() {
            if (!started) return;
            // TODO: read detections and update hasTag/tagId
        }

        public boolean hasTag() { return hasTag; }
        public int getTagId() { return tagId; }
    }
}
