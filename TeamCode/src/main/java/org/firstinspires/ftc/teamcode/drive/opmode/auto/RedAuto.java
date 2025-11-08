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

import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.ColorSensor;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.RobotHeading;
import org.firstinspires.ftc.teamcode.turret.YawServo;

@Autonomous(name = "Red Auto", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class RedAuto extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(144 - 61.5, 9, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose PGPPose = new Pose(144 - 49, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose = new Pose(144 - 44, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Initialize variables for paths

    private PathChain grabPPG;





    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value


    private YawServo yawServo;
    private LaunchMotors launchMotors;
    private LaunchServos launchServos;
    private IntakeServos intakeServos;
    private ColorSensor colorSensor;
    private boolean detected;




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

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        intakeServos = new IntakeServos(hardwareMap, "leftForward", "barFront", "rightForward", "leftBack", "barBack", "rightBack");
        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR", "servoTop");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");
        colorSensor = new ColorSensor(hardwareMap, "color");


        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging
        yawServo.setPosition(0.23);
//        colorSensor.setDelayMillis(350);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathState(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose

            detected = colorSensor.delayedDetection();
            updateStateMachine();

            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }

    //below is the state machine or each pattern

    public void updateStateMachine() {
        switch (pathStatePPG) {
            case 0: // Shoots ball no 1
                presetLaunch(300);
                setpathState(1);
                break;
            case 1: // Moves servo to back intake position, enables back intake
                if (runtime.seconds() > 2){
                    yawServo.back();
                    intakeServos.enableBackIntake();
                    setpathState(2);
                }
                break;
            case 2: // Intakes ball no 2, stops servos when ball reaches color sensor
                if (detected){
                    launchServos.disable();
                }
                if (runtime.seconds() > 4.8) {
                    presetLaunch(300); // Launch ball no 2
                    setpathState(3);
                }
                break;
            case 3: // Moves to front intake position, enables front intake
                if (runtime.seconds() > 6){
                    yawServo.front();
                    sleep(300);
                    intakeServos.enableFrontIntake();
                    setpathState(4);
                }
                break;
            case 4: // Intakes ball no 3, stops servos when ball reaches color sensor
                if (detected){
                    launchServos.disable();
                }
                if (runtime.seconds() > 8.8) {
                    presetLaunch(300); // Shoots ball no 3
                    setpathState(5);
                }
                break;
            case 5: // Turns off launch motors, moves to first artifact pickup pose
                if (runtime.seconds() > 10) {
                    yawServo.back();
                    path1();
                    intakeServos.enableBackIntake();
                    launchServos.enable();
                    setpathState(6);
                }
                break;
            case 6: // Slowly moves to intake balls
                if (runtime.seconds() > 12){
                    path2();
                    setpathState(7);
                }
                break;
            case 7: // Waits until ball is detected to stop intake servos
                if (detected){
                    sleep(200);
                    intakeServos.disableBackWheels();
                    launchServos.disable();
                    setpathState(8);
                }
                break;

            case 8: // Moves to scoring pose
                if (runtime.seconds() > 15.7){
                    launchMotors.set(0.58);
                    path3();
                    setpathState(9);
                }
                break;

            case 9: // Shoot balls no 4 and 5
                if (runtime.seconds() > 18.2 && !follower.isBusy()){ // delay for shooting
                    launchServos.enable();
                    intakeServos.enableBackIntake();
                    setpathState(10);
                }
                break;

            case 10: // Moves to second artifact pickup pose
                if (runtime.seconds() > 20.7){
                    yawServo.back();
                    path4();
                    intakeServos.enableBackIntake();
                    launchServos.enable();
                    setpathState(11);
                }
                break;
            case 11: // Slowly moves to intake balls
                if (runtime.seconds() > 22.2){
                    path5();
                    setpathState(12);
                }
                break;

            case 12: // Waits until ball is detected to stop intake servos
                if (detected){
                    sleep(200);
                    intakeServos.disableBackWheels();
                    launchServos.disable();
                    setpathState(13);
                }
                break;

            case 13: // Moves to scoring pose
                if (runtime.seconds() > 25.5){
                    launchMotors.set(0.575);
                    path6();
                    setpathState(14);
                }
                break;

            case 14: // Shoots ball no 6 and 7
                if (runtime.seconds() > 27.5 && !follower.isBusy()){
                    launchServos.enable();
                    intakeServos.enableBackIntake();
                    setpathState(15);
                }

        }
    }


    // Setter methods for pathState variables placed at the class level
    void setpathState(int newPathState) {
        this.pathStatePPG = newPathState;
    }



    /**
     * start the AprilTag processor.
     */

    private void presetLaunch(int ms) {
        yawServo.setPosition(0.23);
        launchMotors.set(0.65);
        sleep(ms);
        launchServos.enable();

    }


    private void path1(){
        follower.setMaxPower(1);
        grabPPG = follower.pathBuilder() //
                .addPath(new BezierLine(startPose, GPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), GPPPose.getHeading())
                .build();
        follower.followPath(grabPPG);
    }

    private void path2(){
        follower.setMaxPower(0.2);
        PathChain pickupPPG = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, new Pose(144-20, 35.5, Math.toRadians(180))))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), Math.toRadians(180))
                .build();
        follower.followPath(pickupPPG);
    }
    private void path3(){
        follower.setMaxPower(0.5); // move to scoring pos.
        PathChain scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-20, 35.5, Math.toRadians(180)), new Pose(144-48, 48, Math.toRadians(180-116))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-116))
                .build();
        follower.followPath(scorePPG);
    }

    private void path4(){
        follower.setMaxPower(1);
        PathChain scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-48, 48, Math.toRadians(180-116)), PGPPose))
                .setLinearHeadingInterpolation(Math.toRadians(180-116), PGPPose.getHeading())
                .build();
        follower.followPath(scorePGP);
    }

    private void path5(){
        follower.setMaxPower(0.2);
        PathChain pickupPGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, new Pose(144-20, 59.5, Math.toRadians(180))))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), Math.toRadians(180))
                .build();
        follower.followPath(pickupPGP);
    }

    private void path6(){
        follower.setMaxPower(0.5); // move to second scoring pos.
        PathChain scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144-20, 59.5, Math.toRadians(180)), new Pose(144-48, 72, Math.toRadians(180-125))))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-125))
                .build();
        follower.followPath(scorePGP);
    }
}