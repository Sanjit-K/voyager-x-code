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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooter;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@Autonomous(name = "Red Auto", group = "Opmode")
@Configurable
@SuppressWarnings("FieldCanBeLocal")
public class RedAuto extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Pedro / telemetry
    private Follower follower;
    private Pose currentPose;
    private TelemetryManager panelsTelemetry;
    private int lastShootIndex = -1;


    // Robot subsystems
    private BarIntake intake;
    private Shooter shooter;
    private KickerServo kickerServo;
    private Spindexer spindexer;
    private IntakeFlap intakeFlap;
    private DigitalChannel distanceSensor;

    // Paths holder
    private Paths paths;

    // Path state machine
    private int pathState = 0;

    // 3-shot volley state
    private boolean threeShotActive = false;
    private final ElapsedTime threeShotTimer = new ElapsedTime();
    private static final int SHOOT_DELAY = 450;
    private static final int KICK_DELAY = 250;
    private static final int NORMAL_DELAY = 200;
    private static final int FULL_CYCLE = SHOOT_DELAY + KICK_DELAY + NORMAL_DELAY;

    private int volleyStep = 0;
    private final ElapsedTime volleyStepTimer = new ElapsedTime();

    private String shootIndexHistory = "";


    // -------- AprilTag motif detection --------
    private static final int TAG_GPP = 21;
    private static final int TAG_PGP = 22;
    private static final int TAG_PPG = 23;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    // Per-volley shoot orders: which spindex slots to fire in which order
    // 0 = preset, 1 = first cycle, 2 = second cycle, 3 = third cycle
    // Motif type
    private enum Motif {
        GPP, PGP, PPG
    }

    private void setShootIndexOnce(int index) {
        if (lastShootIndex != index) {
            spindexer.setShootIndex(index);
            lastShootIndex = index;

            // Append to history (keep it short)
            shootIndexHistory += " → " + index;
            if (shootIndexHistory.length() > 40) {
                shootIndexHistory = shootIndexHistory.substring(
                        shootIndexHistory.length() - 40
                );
            }

            telemetry.addData("ShootIndex Cmd", index);
            telemetry.addData("ShootIndex Seq", shootIndexHistory);
        }
    }



    // TODO: get from webcam
    private Motif motif = Motif.GPP;  // default

    // volleyOrdersByMotif[motif][volleyIndex][ball#]
    // Fill these with whatever orders you want for each motif.
    // Example values here: GPP matches what you specified earlier.
    private final int[][][] volleyOrdersByMotif = new int[][][]{
            // motif 0: GPP
            {
                    {2, 0, 1}, // preset
                    {2, 0, 1}, // first
                    {1, 0, 2}, // second
                    {0, 1, 2}  // third
            },
            // motif 1: PGP  (placeholder, change to what you want)
            {
                    {0, 2, 1},
                    {0, 2, 1},
                    {0, 1, 2},
                    {2, 0, 1}
            },
            // motif 2: PPG  (placeholder, change to what you want)
            {
                    {0, 1, 2},
                    {0, 1, 2},
                    {2, 0, 1},
                    {0, 1, 2}
            }
    };

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .build();
    }




    // Active orders for the current motif
    private int[][] volleyOrders;      // [volleyIndex][ball# 0..2]
    private int currentVolley = 0;

    private final double SLOW_DOWN = 0.2;



    private int intakeIndex = 0;
    private boolean rightPos = false;
    private boolean spindexAllowed = false;

    // Auto intake / spindexer detection (ported from TeleOp)
    private boolean detected = false;
    private final ElapsedTime detectedTimer = new ElapsedTime();
    private static final int DETECTED_DELAY = 300;
    private String colorLog;
    private boolean motifLocked = false;


    // Logging
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    private void scanMotifAtShoot() {
        if (motifLocked || aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection det : detections) {
            if (det.metadata == null) continue;

            switch (det.id) {
                case TAG_GPP:
                    motif = Motif.GPP;
                    break;
                case TAG_PGP:
                    motif = Motif.PGP;
                    break;
                case TAG_PPG:
                    motif = Motif.PPG;
                    break;
                default:
                    continue;
            }

            // Apply motif
            volleyOrders = volleyOrdersByMotif[motif.ordinal()];
            motifLocked = true;

            telemetry.addData("Motif Locked", motif);
            telemetry.update();

            // Shut down camera
            if (visionPortal != null) {
                visionPortal.close();
                visionPortal = null;
            }

            break;
        }
    }


    @Override
    public void runOpMode() {

        // ---------- Panels + Pedro ----------
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Start pose = first point of shootpreset path
        follower.setStartingPose(new Pose(144-22, 121, Math.toRadians(225)));

        // ---------- Subsystems ----------
        intake = new BarIntake(hardwareMap, "intakeMotor", false);
        shooter = new Shooter(hardwareMap, "shooterMotor", false);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        spindexer = new Spindexer(hardwareMap, "spindexer");
        intakeFlap = new IntakeFlap(hardwareMap, "intakeFlapServo");

        distanceSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        // ---------- Basic startup config ----------
        spindexer.setShootIndex(0);
        kickerServo.normal();
        shooter.setPower(0.8);
        shooter.off();
        intake.stop();
        intakeFlap.off();

        // ---------- Default motif until locked ----------
        motif = Motif.GPP;
        volleyOrders = volleyOrdersByMotif[motif.ordinal()];

        telemetry.addData("Motif (default)", motif);
        telemetry.update();

        // ---------- Build paths ----------
        paths = new Paths(follower);

        log("Status", "Initialized");
        telemetry.update();

        // ---------- WAIT FOR START ----------
        waitForStart();
        if (isStopRequested()) return;

        // ---------- Start camera AFTER start ----------
        initAprilTag();

        // ---------- Reset timers / state ----------
        runtime.reset();
        threeShotTimer.reset();
        detectedTimer.reset();
        setPathState(0);

        // ---------- Main loop ----------
        while (opModeIsActive()) {

            shooter.on();
            intake.spinIntake();
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();

            // -------- automatic advancing spindexer logic --------
            detected = distanceSensor.getState();

            if (detected
                    && !spindexer.isFull()
                    && detectedTimer.milliseconds() > DETECTED_DELAY
                    && spindexAllowed
                    && !threeShotActive
                    && pathState != 1) {

                spindexer.setColorAtPos('#');

                if (!spindexer.isFull()) {
                    spindexer.advanceIntake();
                    rightPos = false;
                } else {
                    setShootIndexOnce(2);
                }

                detectedTimer.reset();
            }
            // -----------------------------------------------------

            updateStateMachine();

            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            log("ShootIndex Seq", shootIndexHistory);
            log("motif", motif);
            telemetry.update();
        }
    }

    // -------- 3-shot volley logic --------

    private void startThreeShotVolley(int volleyIndex) {
        intakeFlap.on();
        currentVolley = volleyIndex;
        spindexAllowed = false;
        threeShotActive = true;

        lastShootIndex = -1;

        volleyStep = 0;
        volleyStepTimer.reset();
        threeShotTimer.reset(); // optional, can remove if not used elsewhere
    }




    private void kickAndClearIndex(int index) {
        spindexer.setColorAtPos('_', index);
        kickerServo.kick();
    }

    private boolean updateThreeShotVolley() {
        if (!threeShotActive) return true;

        int[] order = volleyOrders[currentVolley];
        int firstSlot  = order[0];
        int secondSlot = order[1];
        int thirdSlot  = order[2];

        spindexAllowed = false;
        intake.spinOuttake(0.3);

        double t = volleyStepTimer.milliseconds();

        switch (volleyStep) {

            // ----- Ball 1 -----
            case 0: // setShootIndex phase
                setShootIndexOnce(firstSlot);
                if (t >= SHOOT_DELAY) {
                    volleyStep = 1;
                    volleyStepTimer.reset();
                }
                break;

            case 1: // kick phase
                kickAndClearIndex(firstSlot);
                if (t >= KICK_DELAY) {
                    volleyStep = 2;
                    volleyStepTimer.reset();
                }
                break;

            case 2: // normal phase
                kickerServo.normal();
                if (t >= NORMAL_DELAY) {
                    volleyStep = 3;
                    volleyStepTimer.reset();
                }
                break;

            // ----- Ball 2 -----
            case 3: // setShootIndex phase
                setShootIndexOnce(secondSlot);
                if (t >= SHOOT_DELAY) {
                    volleyStep = 4;
                    volleyStepTimer.reset();
                }
                break;

            case 4: // kick phase
                kickAndClearIndex(secondSlot);
                if (t >= KICK_DELAY) {
                    volleyStep = 5;
                    volleyStepTimer.reset();
                }
                break;

            case 5: // normal phase
                kickerServo.normal();
                if (t >= NORMAL_DELAY) {
                    volleyStep = 6;
                    volleyStepTimer.reset();
                }
                break;

            // ----- Ball 3 -----
            case 6: // setShootIndex phase
                setShootIndexOnce(thirdSlot);
                if (t >= SHOOT_DELAY) {
                    volleyStep = 7;
                    volleyStepTimer.reset();
                }
                break;

            case 7: // kick phase
                kickAndClearIndex(thirdSlot);
                if (t >= KICK_DELAY) {
                    volleyStep = 8;
                    volleyStepTimer.reset();
                }
                break;

            case 8: // normal phase
                kickerServo.normal();
                if (t >= NORMAL_DELAY) {
                    volleyStep = 9;
                    volleyStepTimer.reset();
                }
                break;

            // ----- Done -----
            case 9:
                intakeIndex = 0;
                intake.spinIntake();
                intakeFlap.off();
                spindexer.setIntakeIndex(intakeIndex);
                threeShotActive = false;
                rightPos = false;
                return true;
        }

        return false;
    }




    // -------- path state machine (unchanged logic, uses threeShotActive) --------

    private void updateStateMachine() {
        // Handle volley first
        boolean volleyFinishedThisLoop = false;
        if (threeShotActive) {
            volleyFinishedThisLoop = updateThreeShotVolley();
            if (!volleyFinishedThisLoop) {
                // still shooting, do nothing else this loop
                return;
            }
            // if finished, threeShotActive is now false and we fall through
        }

        switch (pathState) {
            case 0:
                follower.followPath(paths.shootpreset);
                setPathState(1);
                break;

            case 1: // preset volley (shoot position 1)
                if (!follower.isBusy()) {

                    // Scan AprilTag while stationary and aiming
                    scanMotifAtShoot();

                    // 🚨 HARD BLOCK: do NOT shoot until motif is locked
                    if (!motifLocked && !(runtime.milliseconds() > 3000)) {
                        return; // stay here, keep scanning
                    }

                    // Once motif is locked, allow first volley
                    if (!volleyFinishedThisLoop) {
                        startThreeShotVolley(0);
                        return;
                    } else {
                        spindexAllowed = true;
                        follower.followPath(paths.preparepickup1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup1, SLOW_DOWN, false);
                    setPathState(3);
                }
                break;

            case 3:
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    setPathState(5);
                }
                break;

            case 5: // volley after first pickup/gate
                if (!follower.isBusy()) {
                    if (!volleyFinishedThisLoop) {
                        startThreeShotVolley(1);
                        return;
                    } else {
                        spindexAllowed = true;
                        intake.spinIntake();
                        follower.followPath(paths.preparepickup2);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup2, SLOW_DOWN, false);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2);
                    setPathState(8);
                }
                break;

            case 8: // volley after second pickup
                if (!follower.isBusy()) {
                    if (!volleyFinishedThisLoop) {
                        startThreeShotVolley(2);
                        return;
                    } else {
                        spindexAllowed = true;
                        follower.followPath(paths.preparepickup3);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.pickup3, SLOW_DOWN, false);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3);
                    setPathState(11);
                }
                break;

            case 11: // final volley
                if (!follower.isBusy()) {
                    if (!volleyFinishedThisLoop) {
                        startThreeShotVolley(3);
                        return;
                    } else {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ending);
                    setPathState(13);
                }
                break;
        }
    }



    private void setPathState(int newState) {
        pathState = newState;
        runtime.reset();
    }

    // -------- Paths class (unchanged) --------

    public static class Paths {

        public PathChain shootpreset;
        public PathChain preparepickup1;
        public PathChain pickup1;
        public PathChain overflow;
        public PathChain shoot1;
        public PathChain preparepickup2;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain preparepickup3;
        public PathChain pickup3;
        public PathChain shoot3;
        public PathChain ending;

        public Paths(Follower follower) {

            // === shootpreset = line1 ===
            shootpreset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-22.000, 121.000),
                                    new Pose(144-48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(223))
                    .build();

            // === preparepickup1 = line2 ===
            preparepickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-48.000, 96.000),
                                    new Pose(144-61.300, 86.100),
                                    new Pose(144-45.000, 84.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(0))
                    .build();

            // === pickup1 = line3 ===
            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-45.000, 84.500),
                                    new Pose(144-28.000, 84.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // === overflow = line4 (BezierCurve) ===
            overflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-24.000, 84.500),
                                    new Pose(144-28.000, 80.000),
                                    new Pose(144-16.000, 73.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // === shoot1 = line5 ===
            shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-28.000, 84.500),
                                    new Pose(144-48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(223))
                    .build();

            // === preparepickup2 = Path6 (BezierCurve) ===
            preparepickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-48.000, 96.000),
                                    new Pose(144-64.958, 60.029),
                                    new Pose(144-44, 60)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(0))
                    .build();

            // === pickup2 = line7 ===
            pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-44, 60),
                                    new Pose(144-28.000, 60)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // === shoot2 = line8 ===
            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-28.000, 60),
                                    new Pose(144-48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(223))
                    .build();

            // === preparepickup3 = Path9 (BezierCurve) ===
            preparepickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-48.000, 96.000),
                                    new Pose(144-79.746, 31.863),
                                    new Pose(144-44.000, 40)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(223), Math.toRadians(0))
                    .build();

            // === pickup3 = line10 ===
            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-44.000, 40),
                                    new Pose(144-28.000, 40)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // === shoot3 = line11 ===
            shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-28.000, 40),
                                    new Pose(144-48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(223))
                    .build();

            // === ending = Path12 ===
            ending = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-48.000, 96.000),
                                    new Pose(144-48, 70)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(223))
                    .build();
        }
    }




}
