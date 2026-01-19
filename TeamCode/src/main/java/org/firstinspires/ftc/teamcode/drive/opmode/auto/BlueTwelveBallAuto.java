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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;


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

    private int[] order = null;


    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 270;         // turret angle while scanning for tag
    public static double SHOOT_DEG = 315;
    public static double SHOOT_RPM = 2400;

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
    private double lastAdvanceTime = 0.0;



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
                false,
                270
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);   // make sure pipeline 0 is APRILTAG
        limelight.setPollRateHz(100);
        limelight.start();
        spindexer.filled = new char[]{'X', 'X', 'X'};

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
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                scannedTagId = result.getFiducialResults().get(0).getFiducialId();
                order = getMotifForTag(scannedTagId);
                panelsTelemetry.debug("Scanned Tag ID", scannedTagId);
            }
        }

        switch (pathState) {
            // ------------------------------------------------------------
            // 0) Turret to scan angle, start path ShootPreset
            // ------------------------------------------------------------
            case 0:
                follower.followPath(paths.ShootPreset);
                if (order != null && order.length > 0) {
                    spindexer.setShootIndex(order[0]);
                    targetAngle = SHOOT_DEG;
                }
                setState(1);
                break;


            // ------------------------------------------------------------
            // 1) Wait for order (if any) and then wait for follower to finish
            // ------------------------------------------------------------
            case 1:
                if (order != null && order.length > 0 && !outtakeInProgress) {
                    spindexer.setShootIndex(order[0]);
                    targetAngle = SHOOT_DEG;
                }
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() > 5000){
                        startOuttakeRoutine();
                        setState(2);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 2) After outtake completes, go pick up balls
            // ------------------------------------------------------------
            case 2:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup1);
                    if (spindexer.isFull() && !outtakeInProgress) spindexer.setShootIndex(order[1]);
                    setState(3);
                }
                break;

            // ------------------------------------------------------------
            // 3) After pickup1, prepare shoot1
            // ------------------------------------------------------------
            case 3:
                if (!follower.isBusy()) {
                    if (!outtakeInProgress) spindexer.setShootIndex(order[1]);
                    follower.followPath(paths.Shoot1);
                    setState(4);
                }
                break;

            // ------------------------------------------------------------
            // 4) After shoot1 path completes, start outtake
            // ------------------------------------------------------------
            case 4:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(5);
                }
                break;

            // ------------------------------------------------------------
            // 5) After outtake, pickup2
            // ------------------------------------------------------------
            case 5:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup2);
                    if (spindexer.isFull() && !outtakeInProgress) spindexer.setShootIndex(order[2]);
                    setState(6);
                }
                break;

            // ------------------------------------------------------------
            // 6) After pickup2, shoot2
            // ------------------------------------------------------------
            case 6:
                if (!follower.isBusy()) {
                    if (spindexer.isFull() && !outtakeInProgress) spindexer.setShootIndex(order[2]);
                    follower.followPath(paths.Shoot2);
                    setState(7);
                }
                break;

            // ------------------------------------------------------------
            // 7) After shoot2 path completes, start outtake
            // ------------------------------------------------------------
            case 7:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // 8) After outtake, pickup3 (restored missing case)
            // ------------------------------------------------------------
            case 8:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup3);
                    if (spindexer.isFull() && !outtakeInProgress) spindexer.setShootIndex(order[3]);
                    setState(9);
                }
                break;

            // ------------------------------------------------------------
            // 9) After pickup3, shoot3
            // ------------------------------------------------------------
            case 9:
                if (!follower.isBusy()) {
                    if (spindexer.isFull() && !outtakeInProgress) spindexer.setShootIndex(order[3]);
                    follower.followPath(paths.Shoot3);
                    setState(10);
                }
                break;

            // ------------------------------------------------------------
            // 10) After shoot3 path completes, start outtake
            // ------------------------------------------------------------
            case 10:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(11);
                }
                break;

            // ------------------------------------------------------------
            // 11) After final outtake, park
            // ------------------------------------------------------------
            case 11:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(12);
                }
                break;

            // ------------------------------------------------------------
            // 13) Idle
            // ------------------------------------------------------------
            case 12:
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;


        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        // Step 2: Set kicker servo to kick
        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                // All 3 advanceIntake calls completed, set kicker back to normal
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                spindexer.setIntakeIndex(0);
                outtakeInProgress = false;
            }
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

        public PathChain Pickup3;
        public PathChain Shoot3;

        public PathChain Park;

        public Paths(Follower follower) {

            // These match the pathing you showed
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
}
