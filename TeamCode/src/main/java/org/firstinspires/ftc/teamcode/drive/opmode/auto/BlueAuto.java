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

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooter;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@Autonomous(name = "Blue Auto Decode", group = "Opmode")
@Configurable
@SuppressWarnings("FieldCanBeLocal")
public class BlueAuto extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Pedro / telemetry
    private Follower follower;
    private Pose currentPose;
    private TelemetryManager panelsTelemetry;

    // Robot subsystems
    private BarIntake intake;
    private Shooter shooter;
    private KickerServo kickerServo;
    private Spindexer spindexer;
    private DigitalChannel distanceSensor;

    // Paths holder
    private Paths paths;

    // Path state machine
    private int pathState = 0;

    // 3-shot volley state
    private boolean threeShotActive = false;
    private final ElapsedTime threeShotTimer = new ElapsedTime();
    private static final int SHOOT_DELAY = 500;
    private static final int KICK_DELAY = 400;
    private static final int NORMAL_DELAY = 250;
    private static final int FULL_CYCLE = SHOOT_DELAY + KICK_DELAY + NORMAL_DELAY;

    // Per-volley shoot orders: which spindex slots to fire in which order
    // 0 = preset, 1 = first cycle, 2 = second cycle, 3 = third cycle
    // Motif type
    private enum Motif {
        GPP, PGP, PPG
    }

    // TODO: get from webcam
    private Motif motif = Motif.GPP;  // default

    // volleyOrdersByMotif[motif][volleyIndex][ball#]
    // Fill these with whatever orders you want for each motif.
    // Example values here: GPP matches what you specified earlier.
    private final int[][][] volleyOrdersByMotif = new int[][][]{
            // motif 0: GPP
            {
                    {0, 1, 2}, // preset
                    {1, 2, 0}, // first
                    {0, 1, 2}, // second
                    {2, 1, 0}  // third
            },
            // motif 1: PGP  (placeholder, change to what you want)
            {
                    {0, 1, 2},
                    {0, 2, 1},
                    {1, 0, 2},
                    {1, 2, 0}
            },
            // motif 2: PPG  (placeholder, change to what you want)
            {
                    {0, 1, 2},
                    {2, 0, 1},
                    {1, 2, 0},
                    {2, 1, 0}
            }
    };

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

    @Override
    public void runOpMode() {
        // Panels + Pedro
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Start pose = first point of shootpreset path
        follower.setStartingPose(new Pose(13.336, -16.531, 0.963));

        // Subsystems
        intake = new BarIntake(hardwareMap, "intakeMotor", false);
        shooter = new Shooter(hardwareMap, "shooterMotor", false);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        spindexer = new Spindexer(hardwareMap, "spindexer");
        shooter.setPower(0.75);

        distanceSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        // Basic startup config (auto-intaking and spinning shooter)
        spindexer.setShootIndex(0);
        kickerServo.normal();
        shooter.on();
        intake.spinIntake();

        // Build all paths
        paths = new Paths(follower);

        volleyOrders = volleyOrdersByMotif[motif.ordinal()]; //.ordinal() js finds the index coresponding to the motif

        log("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        threeShotTimer.reset();
        detectedTimer.reset();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose();

            // -------- automatic advancing spindexer logic (from TeleOp) --------
            detected = distanceSensor.getState();

            if (detected && !spindexer.isFull()
                    && detectedTimer.milliseconds() > DETECTED_DELAY
                    && !threeShotActive && spindexAllowed) {

                spindexer.setColorAtPos('#');

                if (!spindexer.isFull()) {
                    spindexer.advanceIntake();
                    rightPos = false;
                } else {
                    spindexer.setShootIndex(2);
                }

                detectedTimer.reset();
            }
            // --------------------------------------------------------------------

            updateStateMachine();

            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update();
        }
    }

    // -------- 3-shot volley logic --------

    private void startThreeShotVolley(int volleyIndex) {
        currentVolley = volleyIndex;   // which of the 4 patterns (0..3)
        threeShotActive = true;
        threeShotTimer.reset();
    }



    private void kickAndClearIndex(int index) {
        spindexer.setColorAtPos('_', index);
        kickerServo.kick();
    }

    private boolean updateThreeShotVolley() {
        if (!threeShotActive) return true;

        double t = threeShotTimer.milliseconds();
        int fd = rightPos ? SHOOT_DELAY : 0;

        // Get order for this volley and motif
        int[] order = volleyOrders[currentVolley];
        int firstSlot  = order[0];
        int secondSlot = order[1];
        int thirdSlot  = order[2];
        spindexAllowed = false;



        intake.spinOuttake();

        // ball 1
        if (t > 0 && t < SHOOT_DELAY - fd) {
            spindexer.setShootIndex(firstSlot);
        }
        if (t > SHOOT_DELAY - fd && t < SHOOT_DELAY + KICK_DELAY - fd) {
            kickAndClearIndex(firstSlot);
        }
        if (t > SHOOT_DELAY + KICK_DELAY - fd && t < FULL_CYCLE - fd) {
            kickerServo.normal();
        }

        // ball 2
        if (t > FULL_CYCLE - fd && t < FULL_CYCLE + SHOOT_DELAY - fd) {
            spindexer.setShootIndex(secondSlot);
        }
        if (t > FULL_CYCLE + SHOOT_DELAY - fd && t < 2 * FULL_CYCLE - NORMAL_DELAY - fd) {
            kickAndClearIndex(secondSlot);
        }
        if (t > 2 * FULL_CYCLE - NORMAL_DELAY - fd && t < 2 * FULL_CYCLE - fd) {
            kickerServo.normal();
        }

        // ball 3
        if (t > 2 * FULL_CYCLE - fd && t < 2 * FULL_CYCLE + SHOOT_DELAY - fd) {
            spindexer.setShootIndex(thirdSlot);
        }
        if (t > 2 * FULL_CYCLE + SHOOT_DELAY - fd && t < 3 * FULL_CYCLE - NORMAL_DELAY - fd) {
            kickAndClearIndex(thirdSlot);
        }
        if (t > 3 * FULL_CYCLE - NORMAL_DELAY - fd && t < 3 * FULL_CYCLE - fd) {
            kickerServo.normal();
        }

        if (t > 3 * FULL_CYCLE + 200 - fd) {
            intakeIndex = 0;
            intake.spinIntake();
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

            case 1: // preset volley
                if (!follower.isBusy()) {
                    if (!volleyFinishedThisLoop) {
                        // have not shot yet in this state → start volley 0
                        startThreeShotVolley(0);
                        return;
                    } else {
                        // volley 0 just finished this loop → move to lane 1
                        spindexAllowed = true;
                        follower.setMaxPower(0.8);
                        follower.followPath(paths.preparepickup1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(SLOW_DOWN);
                    follower.followPath(paths.pickup1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {

                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
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
                        follower.followPath(paths.preparepickup2);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(SLOW_DOWN);
                    follower.followPath(paths.pickup2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
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
                    follower.setMaxPower(SLOW_DOWN);
                    follower.followPath(paths.pickup3);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
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
                // done
                break;
        }
    }



    private void setPathState(int newState) {
        pathState = newState;
        runtime.reset();
    }

    // -------- Paths class (unchanged) --------

    public static class Paths {

        // ===== Tunable poses =====

        // Start of auto
        public static final Pose START_POSE = new Pose(
                13.336, -16.531, 0.963
        );

        // Common shooting pose
        public static final Pose SHOOT_POSE = new Pose(
                37.82, 13.98, 0.88516
        );

        // All prepares share this X
        public static final double PREPARE_Y = 10; // how far back from balls u start

        // All pickups share this X
        public static final double PICKUP_Y  = -11;   // how forward into balls u drive

        // Lanes for each of the 3 sets
        public static final double LANE1_X = 49;
        public static final double LANE2_X = 76;
        public static final double LANE3_X = 99;

        // Derived poses for each lane (prepare from SHOOT_POSE -> PREPARE_X,LANE_Y; pickup from PREPARE_X,LANE_Y -> PICKUP_X,LANE_Y)
        public static final Pose PREPARE1 = new Pose(
                LANE1_X, PREPARE_Y, -1.56
        );
        public static final Pose PICKUP1_END = new Pose(
                LANE1_X, PICKUP_Y, -1.56
        );

        public static final Pose PREPARE2 = new Pose(
                LANE2_X, PREPARE_Y, -1.56
        );
        public static final Pose PICKUP2_END = new Pose(
                LANE2_X, PICKUP_Y, -1.56
        );

        public static final Pose PREPARE3 = new Pose(
                LANE3_X, PREPARE_Y, -1.56
        );
        public static final Pose PICKUP3_END = new Pose(
                LANE3_X, PICKUP_Y, -1.56
        );

        // Gate hit path
        public static final Pose GATE_CTRL = new Pose(
                59.314, 81.429, 0
        );
        public static final Pose GATE_POSE = new Pose(
                15.257, 71.657, Math.toRadians(90)
        );

        // ===== PathChains =====

        public PathChain shootpreset;
        public PathChain preparepickup1;
        public PathChain pickup1;
        public PathChain releasegate;
        public PathChain shoot1;
        public PathChain preparepickup2;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain preparepickup3;
        public PathChain pickup3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            // From start to first shooting pose
            shootpreset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    START_POSE,
                                    SHOOT_POSE
                            )
                    )
                    .setLinearHeadingInterpolation(
                            START_POSE.getHeading(),
                            SHOOT_POSE.getHeading()
                    )
                    .build();

            // Lane 1: SHOOT_POSE -> prepare1 -> pickup1
            preparepickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(SHOOT_POSE, PREPARE1)
                    )
                    .setLinearHeadingInterpolation(
                            SHOOT_POSE.getHeading(),
                            PREPARE1.getHeading()
                    )
                    .build();

            pickup1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(PREPARE1, PICKUP1_END)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Swing out and hit gate from lane 1
            releasegate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    PICKUP1_END,
                                    GATE_CTRL,
                                    GATE_POSE
                            )
                    )
                    .setLinearHeadingInterpolation(
                            PICKUP1_END.getHeading(),
                            GATE_POSE.getHeading()
                    )
                    .build();

            // Back to shooting pose from gate
            shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(GATE_POSE, SHOOT_POSE)
                    )
                    .setLinearHeadingInterpolation(
                            GATE_POSE.getHeading(),
                            SHOOT_POSE.getHeading()
                    )
                    .build();

            // Lane 2: SHOOT_POSE -> prepare2 -> pickup2
            preparepickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(SHOOT_POSE, PREPARE2)
                    )
                    .setLinearHeadingInterpolation(
                            SHOOT_POSE.getHeading(),
                            PREPARE2.getHeading()
                    )
                    .build();

            pickup2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(PREPARE2, PICKUP2_END)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Back to shooting pose from lane 2
            shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(PICKUP2_END, SHOOT_POSE)
                    )
                    .setLinearHeadingInterpolation(
                            PICKUP2_END.getHeading(),
                            SHOOT_POSE.getHeading()
                    )
                    .build();

            // Lane 3: SHOOT_POSE -> prepare3 -> pickup3
            preparepickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(SHOOT_POSE, PREPARE3)
                    )
                    .setLinearHeadingInterpolation(
                            SHOOT_POSE.getHeading(),
                            PREPARE3.getHeading()
                    )
                    .build();

            pickup3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(PREPARE3, PICKUP3_END)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Back to shooting pose from lane 3
            shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(PICKUP3_END, SHOOT_POSE)
                    )
                    .setLinearHeadingInterpolation(
                            PICKUP3_END.getHeading(),
                            SHOOT_POSE.getHeading()
                    )
                    .build();
        }
    }


}
