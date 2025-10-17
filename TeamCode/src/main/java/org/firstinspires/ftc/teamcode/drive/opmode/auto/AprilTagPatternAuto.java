/*
Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
that when you scan them they give u a numerical value. the april tag values for this season are as the following-

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/


package org.firstinspires.ftc.teamcode.drive.opmode.auto;

// FTC SDK

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.AprilTagScanner;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Supplier;

@Autonomous(name = "April Tag with PP test", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class AprilTagPatternAuto extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(63, 33, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose PPGPose = new Pose(44, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(44, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(44, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Initialize variables for paths
    private PathChain grabPPG;
    private PathChain scorePPG;
    private PathChain grabPGP;
    private PathChain scorePGP;
    private PathChain grabGPP;
    private PathChain scoreGPP;

    //set April Tag values to specific patterns
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    // AprilTag scanner with improved detection
    private AprilTagScanner aprilTagScanner;

    // Other variables
    private DcMotorEx motorL, motorR;
    private double turretPower = 0.5;
    private boolean turretSpinning = false;

    // Intake servos
    private IntakeServos intakeServos;
    private LaunchServos launchServos;
    private LaunchMotors launchMotors;

    // Turret helpers
    private YawServo yawServo;
    private TurretConstants turretConstants;
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value
    private int foundID = -1; // Current state machine value, dictates which one to run


    // Custom logging function to support telemetry and Panels
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

    // a place to put your intake and shooting functions


    public void shootArtifacts() {
        // Put your shooting logic/functions here
    }


    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize hardware
        motorL = hardwareMap.get(DcMotorEx.class, "turretL");
        motorR = hardwareMap.get(DcMotorEx.class, "turretR");

        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turret initialization
        turretConstants = new TurretConstants();
        try {
            yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        } catch (Exception e) {
            yawServo = null;
            log("Warning", "Yaw servo not found in hardware map");
        }

        // Initialize AprilTag scanner with improved detection
        aprilTagScanner = new AprilTagScanner(hardwareMap, "Webcam 1");

        // Initialize Intake Servos
        intakeServos = new IntakeServos(hardwareMap, "servo1", "servo2", "servo3", "servo4", "servo5", "servo6", "servo7", "servo8");
        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");
        launchMotors.set(0.75);
        intakeServos.enableAllIntake();
        launchServos.enable();

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        log("Camera FPS", aprilTagScanner.getFps());
        log("Camera State", aprilTagScanner.getCameraState());
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            // Get detections from the improved scanner
            List<AprilTagDetection> currentDetections = aprilTagScanner.getDetections();

            // Only scan for tags if we haven't found one yet
            if (foundID == -1 && !currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if (detection.id == PPG_TAG_ID) {
                            // call lines for the PPG pattern
                            buildPathsPPG();
                            foundID = PPG_TAG_ID;
                            log("Detected Tag", "PPG (23)");
                            break;  // don't look any further.
                        } else if (detection.id == PGP_TAG_ID) {
                            // call lines for the PGP pattern
                            buildPathsPGP();
                            foundID = PGP_TAG_ID;
                            log("Detected Tag", "PGP (22)");
                            break;  // don't look any further.
                        } else if (detection.id == GPP_TAG_ID) {
                            // call lines for the GPP pattern
                            buildPathsGPP();
                            foundID = GPP_TAG_ID;
                            log("Detected Tag", "GPP (21)");
                            break;  // don't look any further.
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }
            }

            // Update the state machine
            if (foundID == PPG_TAG_ID) {
                updateStateMachinePPG();
            } else if (foundID == PGP_TAG_ID) {
                updateStateMachinePGP();
            } else if (foundID == GPP_TAG_ID) {
                updateStateMachineGPP();
            }

            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            log("Camera FPS", aprilTagScanner.getFps());
            log("Tags Detected", currentDetections.size());
            if (foundID != -1) {
                log("Tracking Pattern", foundID == PPG_TAG_ID ? "PPG" : foundID == PGP_TAG_ID ? "PGP" : "GPP");
            }
            telemetry.update(); // Update the driver station after logging
        }

        // Clean up when done
        aprilTagScanner.close();
    }


    public void buildPathsPPG() {
        // basically just plotting the points for the lines that score the PPG pattern


        grabPPG = follower.pathBuilder() //
                .addPath(new BezierLine(startPose, PPGPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
//        scorePPG = follower.pathBuilder()
//                .addPath(new BezierLine(PPGPose, scorePose))
//                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
//                .build();
    }

    public void buildPathsPGP() {
        // basically just plotting the points for the lines that score the PGP pattern

        // Move to the first artifact pickup pose from the start pose
        grabPGP = follower.pathBuilder() // Changed from scorePGP to grabPGP
                .addPath(new BezierLine(startPose, PGPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
//        scorePGP = follower.pathBuilder()
//                .addPath(new BezierLine(PGPPose, scorePose))
//                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
//                .build();
    }

    public void buildPathsGPP() {
        // basically just plotting the points for the lines that score the GPP pattern

        // Move to the first artifact pickup pose from the start pose
        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
//        scoreGPP = follower.pathBuilder()
//                .addPath(new BezierLine(GPPPose, scorePose))
//                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
//                .build();
    }

    //below is the state machine or each pattern

    public void updateStateMachinePPG() {
        switch (pathStatePPG) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabPPG);
                setpathStatePPG(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the first artifact pickup location from the scoring position
                    intake2();
                    setpathStatePPG(-1); //set it to -1 so it stops the state machine execution
                }
                break;



        }
    }


    public void updateStateMachinePGP() {
        switch (pathStatePGP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabPGP);
                setpathStatePGP(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(scorePGP);
                    setpathStatePGP(-1); // Call the setter for PGP
                }
                break;
        }
    }


    public void updateStateMachineGPP() {
        switch (pathStateGPP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(grabGPP);
                setpathStateGPP(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(scoreGPP);
                    setpathStateGPP(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }

    public void intake2() {
        switch (foundID) {
            case PPG_TAG_ID:
                Pose intake1 = new Pose(34, 83.5, Math.toRadians(180));

                // Define slower path constraints for precise intake


                // Build the path
                PathChain intake1PPG = follower.pathBuilder()
                        .addPath(new BezierLine(PPGPose, intake1))
                        .setLinearHeadingInterpolation(PPGPose.getHeading(), intake1.getHeading())
                        .build();

                follower.setMaxPower(0.3);
                // Follow the path with slower constraints
                follower.followPath(intake1PPG);


                break;
            case PGP_TAG_ID:
                shootArtifacts();
                break;
            case GPP_TAG_ID:
                shootArtifacts();
                break;
            default:
                log("Error", "Unknown pattern: " + foundID);
                break;
        }
    }

    // Setter methods for pathState variables placed at the class level
    void setpathStatePPG(int newPathState) {
        this.pathStatePPG = newPathState;
    }

    void setpathStatePGP(int newPathState) {
        this.pathStatePGP = newPathState;
    }

    void setpathStateGPP(int newPathState) {
        this.pathStateGPP = newPathState;
    }
}

