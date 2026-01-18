package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

    // -------------------- Motifs --------------------
    private Limelight3A limelight;
    private int scannedTagId = 0;

    // Simple motif lookup helpers. Given a tag id (21,22,23) return a 4-int pattern
    // The integers are placeholders (0/1/2) and can be changed later.
    private static int[] getMotifForTag(int tagId) {
        switch (tagId) {
            case 21: return new int[]{0, 0, 2, 1};
            case 22: return new int[]{1, 1, 0, 2};
            case 23: return new int[]{2, 2, 1, 0};
            default: return null;
        }
    }

    private int[] order;


    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 308;         // turret angle while scanning for tag
    public static double SHOOT_DEG = 308;
    public static double SHOOT_RPM = 2750;

    public static double PARK_SPEED = 0.50;         // follower speed scalar for park

    // Outtake cadence
    public static double OUTTAKE_DELAY_MS = 700;
    private double targetAngle = SCAN_TURRET_DEG;

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
    private double lastAdvanceTimeMs = 0.0;



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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);   // make sure pipeline 0 is APRILTAG
        limelight.setPollRateHz(100);
        limelight.start();

        // Paths
        paths = new Paths(follower);

        // Startup config
        kickerServo.normal();
        turret.setShooterRPM(SHOOT_RPM);



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
        if (!outtakeInProgress && spindexer.isFull()) {
            barIntake.stop();
        }

        // 4) Run state machine
        autonomousUpdate();

        // 5) Telemetry
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());


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

        if (scannedTagId == 0) {
            // Check for tag detection
            scannedTagId = limelight.getLatestResult().getFiducialResults().get(0).getFiducialId();
            order = getMotifForTag(scannedTagId);
            panelsTelemetry.debug("Scanned Tag ID", scannedTagId);
        }

        switch (pathState) {

            // ------------------------------------------------------------
            // 0) Turret to scan angle, start path ShootPreset
            // ------------------------------------------------------------
            case 0:
                follower.followPath(paths.ShootPreset);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Lock a tag decision here (read once at the shoot location)

                    // Optional small settle
                    targetAngle = SHOOT_DEG;
                    if (stateTimer.milliseconds() < 500) return;

                    startOuttakeRoutine();
                    setState(2);
                }
                break;

            case 2:
                if (!outtakeInProgress) {
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
                    startOuttakeRoutine();
                    setState(5);
                }
                break;

            case 5:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(6);
                }
                break;

            case 7:
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        // Kick
        kickerServo.kick();

        // First advance immediately
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTimeMs = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double now = outtakeTimer.milliseconds();

        if (outtakeAdvanceCount < 3) {
            if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTimeMs = now;
            }
            return;
        }

        // After the 3 advances, wait one more delay then finish
        if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
            kickerServo.normal();
            spindexer.clearTracking();
            barIntake.spinIntake();
            outtakeInProgress = false;
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
