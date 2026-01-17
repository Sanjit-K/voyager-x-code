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

import org.firstinspires.ftc.teamcode.AprilTagScanner;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

    // -------------------- AprilTag Scanner (REAL) --------------------
    private AprilTagScanner aprilTagScanner;
    private Integer frozenTagId = null; // "lock in" the best tag id at the shoot location

    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 308;         // turret angle while scanning for tag
    public static double DEFAULT_SHOOT_TURRET_DEG = 286; // fallback angle if tag not seen
    public static double SHOOT_RPM = 2750;

    public static double PARK_SPEED = 0.70;         // follower speed scalar for park
    public static int WAIT_AFTER_SHOOT2_MS = 200;   // requested settle wait

    // Outtake cadence
    public static double OUTTAKE_DELAY_MS = 700;
    private double targetAngle = 308;

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

        // IMPORTANT: starting pose must match start of first path
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
        turret.setShooterRPM(SHOOT_RPM);

//        aprilTagScanner = new AprilTagScanner(hardwareMap, "limelight");


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        frozenTagId = null;
        setState(0);

        stateTimer.reset();

        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);
    }

    @Override
    public void loop() {
        // 1) Always update follower first
        follower.update();

        // 2) Always keep shooter ready
        turret.on();
        turret.setShooterRPM(SHOOT_RPM);
        turret.goToPosition(targetAngle);

        // 3) Update spindexer and run motif classification
        spindexer.update();

        // 4) Run state machine
        autonomousUpdate();

        // 5) Telemetry
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", safeGetBalls());

//        boolean seen = (aprilTagScanner != null && aprilTagScanner.hasDetections());
//        panelsTelemetry.debug("Tag Seen", seen);
//        panelsTelemetry.debug("Frozen Tag", frozenTagId == null ? -1 : frozenTagId);
//        panelsTelemetry.debug("Tag Count", aprilTagScanner == null ? 0 : aprilTagScanner.getDetectionCount());
//        panelsTelemetry.debug("Cam FPS", aprilTagScanner == null ? 0 : aprilTagScanner.getCameraFPS());

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
//        if (aprilTagScanner != null) {
//            aprilTagScanner.close();
//        }
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
                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Lock a tag decision here (read once at the shoot location)
//                    frozenTagId = getBestVisibleTagId();

                    // Optional small settle
                    targetAngle = DEFAULT_SHOOT_TURRET_DEG;
                    if (stateTimer.milliseconds() < 150) return;

                    outtakeTargetShots = 3;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(2);
                }
                break;

            case 2:
                if (!outtakeInProgress) {
                    barIntake.spinIntake();
                    follower.followPath(paths.Pickup2);
                    setState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    setState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < WAIT_AFTER_SHOOT2_MS) return;

                    double shootAngle = 300;
                    turret.goToPosition(shootAngle);

                    outtakeTargetShots = 2;
                    startOuttakeRoutine(outtakeTargetShots);
                    setState(5);
                }
                break;

            case 5:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setState(7);
                }
                break;

            case 7:
                break;
        }
    }

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

    // -----------------------------------------------------------------------------------------
    // AprilTag -> choose angle selection
    // -----------------------------------------------------------------------------------------




    // -----------------------------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------------------------

    private int safeGetBalls() {
        try {
            return spindexer.getBalls();
        } catch (Throwable t) {
            return -1;
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
}
