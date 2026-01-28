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
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

import java.util.Objects;


@Autonomous(name = "Red 12 Ball Auto", group = "Autonomous")
@Configurable
public class RedTwelveBallAuto extends OpMode {

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
            case 21: return new int[]{2, 2, 1, 0};
            case 22: return new int[]{1, 1, 0, 2};
            case 23: return new int[]{0, 0, 2, 1};
            default: return null;
        }
    }

    private int[] order = null;
    private int currentOrderIndex = 0;
    private String currentBarIntakeState = "stop";


    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 110;         // turret angle while scanning for tag
    public static double SHOOT_DEG = 43.5;
    public static double SHOOT_RPM = 2140;

    public static double PARK_SPEED = 1.0;         // follower speed scalar for park

    // Outtake cadence
    public static double OUTTAKE_DELAY_MS =  700;
    private double targetAngle = SCAN_TURRET_DEG;

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // -- New delay logic --
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;
    private static final long SETTLE_DELAY_MS = 300;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false; // Reset settling flag on state change
        }
        pathState = s;
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0.0;
    private int spinInterval = 60;



    // -----------------------------------------------------------------------------------------
    // init / start / loop
    // -----------------------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // IMPORTANT: starting pose must match start of first path
        follower.setStartingPose(new Pose(144-21.5, 122.5, Math.toRadians(0)));

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
                false
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
        if ((spindexer.isFull() && !outtakeInProgress) && follower.getPose().getX() < 132){
            spinInterval++;
            if ((spinInterval > 30 && spinInterval < 50))
                currentBarIntakeState = "out";
            else {
                currentBarIntakeState = "stop";
            }
        }

        if(follower.getPose().getX() > 132 || pathState == 2 || pathState == 6 || pathState == 9){
            currentBarIntakeState = "in";
        }

        // 4) Run state machine
        autonomousUpdate();
        PoseStorage.currentPose = follower.getPose();

        // 5) Telemetry
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());
        panelsTelemetry.debug("Scanned Tag ID", scannedTagId);
        if(currentBarIntakeState.equals("in")){
            barIntake.spinIntake();
        }else if(currentBarIntakeState.equals("out")){
            barIntake.spinIntake();
        }else{
            barIntake.stop();
        }



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
            }
        }

        // Global check for spindexer full status
        if (order != null && currentOrderIndex < order.length && !outtakeInProgress) {
            targetAngle = SHOOT_DEG;
            if (spindexer.isFull()) {
                spindexer.setShootIndex(order[currentOrderIndex]);
            }
        }

        switch (pathState) {
            // ------------------------------------------------------------
            // 0) Turret to scan angle, start path ShootPreset
            // ------------------------------------------------------------
            case 0:
                follower.followPath(paths.ShootPreset);
                setState(1);
                break;


            // ------------------------------------------------------------
            // 1) Wait for order (if any) and then wait for follower to finish
            // ------------------------------------------------------------
            case 1:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2000) {
                    startOuttakeRoutine();
                    setState(2);
                    currentOrderIndex = 1;
                }
                break;

            // ------------------------------------------------------------
            // 2) After outtake completes, go pick up balls
            // ------------------------------------------------------------
            case 2:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup1);
                    setState(3);
                }
                break;


            case 3:
                if (!follower.isBusy()){
                    follower.followPath(paths.Overflow);
                    setState(4);
                }

                // ------------------------------------------------------------
                // 3) After pickup1, prepare shoot1
                // ------------------------------------------------------------
            case 4:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        spinInterval = 10;
                        follower.followPath(paths.Shoot1);
                        spindexer.setShootIndex(order[currentOrderIndex]);
                        setState(5);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 4) After shoot1 path completes, start outtake
            // ------------------------------------------------------------
            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(6);
                        currentOrderIndex = 2;
                    }
                }
                break;

            // ------------------------------------------------------------
            // 5) After outtake, pickup2
            // ------------------------------------------------------------
            case 6:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup2);
                    setState(7);
                }
                break;


            // ------------------------------------------------------------
            // 7) After overflow, shoot2
            // ------------------------------------------------------------
            case 7:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        spinInterval = 10;
                        follower.followPath(paths.Shoot2);
                        spindexer.setShootIndex(order[currentOrderIndex]);
                        setState(8);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 8) After shoot2 path completes, start outtake
            // ------------------------------------------------------------
            case 8:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out") ) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(9);
                        currentOrderIndex = 3;
                    }
                }
                break;

            // ------------------------------------------------------------
            // 9) After outtake, pickup3
            // ------------------------------------------------------------
            case 9:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup3);
                    setState(10);
                }
                break;

            // ------------------------------------------------------------
            // 10) After pickup3, shoot3
            // ------------------------------------------------------------
            case 10:
                if (!follower.isBusy()){
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        spinInterval = 0;
                        follower.followPath(paths.Shoot3);
                        spindexer.setShootIndex(order[currentOrderIndex]);
                        setState(11);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 11) After shoot3 path completes, start outtake
            // ------------------------------------------------------------
            case 11:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(12);
                    }
                }
                break;

            // ------------------------------------------------------------
            // 12) After final outtake, park
            // ------------------------------------------------------------
            case 12:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(13);
                }
                break;

            // ------------------------------------------------------------
            // 13) Idle
            // ------------------------------------------------------------
            case 13:
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
        currentBarIntakeState = "stop";

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
                currentBarIntakeState = "in";
                spindexer.setIntakeIndex(0);
                spinInterval = 0;
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
        public PathChain Overflow;
        public PathChain Shoot1;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Pickup3;
        public PathChain Shoot3;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootPreset = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-21.500, 122.500),
                                    new Pose(144-38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-38.000, 108.000),
                                    new Pose(144-67.500, 79.000),
                                    new Pose(144-13.000, 84.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Overflow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-13.000, 84.500),
                                    new Pose(144-37.000, 76.000),
                                    new Pose(144-14.500, 76.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-14.500, 76.000),
                                    new Pose(144-38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-38.000, 108.000),
                                    new Pose(144-88.000, 55.000),
                                    new Pose(144-5.500, 59.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-5.500, 59.500),
                                    new Pose(144-61.000, 52.000),
                                    new Pose(144-38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-38.000, 108.000),
                                    new Pose(144-86.000, 27.500),
                                    new Pose(144-5.500, 35.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144-5.500, 35.500),
                                    new Pose(144-40, 48.501),
                                    new Pose(144-3.191, 75.649),
                                    new Pose(144-52.419, 68.354),
                                    new Pose(144-32.928, 93.919),
                                    new Pose(144-38.000, 108.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-38.000, 108.000),
                                    new Pose(144-38.000, 75.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
        }
    }


}
