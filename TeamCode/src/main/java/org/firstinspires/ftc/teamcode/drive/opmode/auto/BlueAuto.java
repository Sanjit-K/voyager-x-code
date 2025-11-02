
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
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.ColorSensor;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.RobotHeading;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Blue Auto", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class BlueAuto extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(63, 9, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 20, Math.toRadians(115)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose PPGPose = new Pose(44, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose = new Pose(49, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(44, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Initialize variables for paths

    private PathChain grabPPG;
    private PathChain scorePPG;
    private PathChain grabPGP;
    private PathChain scorePGP;
    private PathChain grabGPP;
    private PathChain scoreGPP;




    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value

    private YawServo yawServo;
    private LaunchMotors launchMotors;
    private LaunchServos launchServos;
    private IntakeServos intakeServos;
    private RobotHeading robotHeading;
    private ColorSensor colorSensor;
    private boolean detected;

    private int foundID; // Current state machine value, dictates which one to run



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
    public void intakeArtifacts() {
        // Put your intake logic/functions here
    }

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

        yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        intakeServos = new IntakeServos(hardwareMap, "leftForward", "barFront", "rightForward", "leftBack", "barBack", "rightBack");
        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");
        colorSensor = new ColorSensor(hardwareMap, "color");

        robotHeading = new RobotHeading(follower);

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            detected = colorSensor.detection();
            updateStateMachine();



            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }


    public void buildPathsPPG() {
        // basically just plotting the points for the lines that score the PPG pattern


        grabPPG = follower.pathBuilder() //
                .addPath(new BezierLine(startPose, PPGPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PPGPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, scorePose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsPGP() {
        // basically just plotting the points for the lines that score the PGP pattern

        // Move to the first artifact pickup pose from the start pose
        grabPGP = follower.pathBuilder() // Changed from scorePGP to grabPGP
                .addPath(new BezierLine(startPose, PGPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), PGPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, scorePose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void buildPathsGPP() {
        // basically just plotting the points for the lines that score the GPP pattern

        // Move to the first artifact pickup pose from the start pose
        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();

        // Move to the scoring pose from the first artifact pickup pose
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, scorePose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                .build();
    }

    //below is the state machine or each pattern

    public void updateStateMachine() {
        switch (pathStatePPG) {
            case 0:
                presetLaunch(500);
                setpathStatePPG(1);
                break;// Call the setter method
            case 1:
                if (runtime.seconds() > 2){
                    yawServo.back();
                    intakeServos.enableBackIntake();
                    setpathStatePPG(2); // Call the setter method
                }
                break;
            case 2:
                if (detected){
                    launchServos.disable();
                }
                if (runtime.seconds() > 4.75) {
                    presetLaunch(300);
                    setpathStatePPG(3);
                }
                break;
            case 3:
                if (runtime.seconds() > 6){
                    yawServo.front();
                    sleep(300);
                    intakeServos.enableFrontIntake();
                    setpathStatePPG(4);
                }
                break;
            case 4:
                if (detected){
                    launchServos.disable();
                }
                if (runtime.seconds() > 8.75) {
                    presetLaunch(200);
                    setpathStatePPG(5);
                }
                break;
            case 5:
                if (runtime.seconds() > 10){
                    launchMotors.set(0.0);
                    yawServo.front();
                    path1();
                    intakeServos.enableFrontIntake();
                    launchServos.enable();
                    setpathStatePPG(6);
                }
                break;
            case 6:
                if (runtime.seconds() > 12){
                    path2();
                    setpathStatePPG(7);
                }
                break;
            case 7:
                if (detected){
                    intakeServos.disableFrontWheels();
                    launchServos.disable();
                    setpathStatePPG(8);
                }
                break;

            case 8:
                if (runtime.seconds() > 14.5){
                    launchMotors.set(0.54);
                    path3();
                    setpathStatePPG(9);
                }
                break;

            case 9:
                if (runtime.seconds() > 17){
                    launchServos.enable();
                    launchMotors.set(0.55);
                    intakeServos.enableFrontIntake();
                    setpathStatePPG(10);
                }
                break;

            case 10:
                if (runtime.seconds() > 20){
                    launchMotors.set(0.0);
                    yawServo.front();
                    path4();
                    intakeServos.enableFrontIntake();
                    launchServos.enable();
                    setpathStatePPG(11);
                }
                break;
            case 11:
                if (runtime.seconds() > 22){
                    path5();
                    setpathStatePPG(12);
                }
                break;

            case 12:
                if (detected){
                    intakeServos.disableFrontWheels();
                    launchServos.disable();
                    setpathStatePPG(13);
                }
                break;

            case 13:
                if (runtime.seconds() > 24.5){
                    launchMotors.set(0.52);
                    path6();
                    setpathStatePPG(14);
                }
                break;

            case 14:
                if (runtime.seconds() > 27){
                    launchServos.enable();
                    launchMotors.set(0.53);
                    intakeServos.enableFrontIntake();
                    setpathStatePPG(15);
                }



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



    /**
     * start the AprilTag processor.
     */

    private void presetLaunch(int ms) {
        yawServo.setPosition(0.36);
        launchMotors.set(0.6);
        sleep(ms);
        launchServos.enable();

    }


    private void path1(){
        grabPPG = follower.pathBuilder() //
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();
        follower.followPath(grabPPG);
    }

    private void path2(){
        follower.setMaxPower(0.15);
         PathChain pickupPPG = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, new Pose(20, 35.5, Math.toRadians(180))))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), Math.toRadians(180))
                .build();
        follower.followPath(pickupPPG);
    }
    private void path3(){
        follower.setMaxPower(0.5);
        PathChain scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20, 35.5, Math.toRadians(180)), new Pose(48, 48, Math.toRadians(180+110))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180+110))
                .build();
        follower.followPath(scorePPG);
    }

    private void path4(){
        follower.setMaxPower(1);
        PathChain scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48, 48, Math.toRadians(180+110)), PGPPose))
                .setLinearHeadingInterpolation(Math.toRadians(180+110), PGPPose.getHeading())
                .build();
        follower.followPath(scorePGP);
    }

    private void path5(){
        follower.setMaxPower(0.15);
        PathChain pickupPGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, new Pose(20, 59.5, Math.toRadians(180))))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), Math.toRadians(180))
                .build();
        follower.followPath(pickupPGP);
    }

    private void path6(){
        follower.setMaxPower(0.5);
        PathChain scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20, 59.5, Math.toRadians(180)), new Pose(48, 72, Math.toRadians(180+125))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180+125))
                .build();
        follower.followPath(scorePGP);
    }
}