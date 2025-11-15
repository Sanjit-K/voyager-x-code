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

@Autonomous(name = "Blue Preset Auto", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class BluePresetAuto extends LinearOpMode {
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(33, 135, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose backPose = new Pose(60, 135, Math.toRadians(180)); // Position to move back to

    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value
    private PathChain moveBackPath; // Path to move back to position


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

        // Build the path to move back to position (60, 135, 180)
        moveBackPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, backPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), backPose.getHeading())
                .build();

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging
        yawServo.back();
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
            case 0: // Move back to position (60, 135, 180)
                follower.followPath(moveBackPath);
                setpathState(1);
                break;
            case 1: // Wait for path to complete
                if (!follower.isBusy()) {
                    setpathState(2);
                }
                break;
            case 2: // Start launch motors
                launchMotors.set(0.55);
                setpathState(3);
                break;
            case 3: // Enable launch servos and back intake
                if (runtime.seconds() > 3){
                    launchServos.enable();
                    intakeServos.enableBackIntake();
                    setpathState(4);
                }
                break;
            case 4: // Disable back intake and rotate to front
                if (runtime.seconds() > 7){
                   intakeServos.disableBackIntake();
                    yawServo.front();
                    setpathState(5);
                }
                break;
            case 5: // Enable front intake
                if (runtime.seconds() > 8){
                    intakeServos.enableFrontIntake();
                    setpathState(6);
                }
                break;
            case 6: // Wait for detection
                if (detected) {
                    launchServos.disable();
                    setpathState(7);
                }
                break;
            case 7: // Rotate back
                if (runtime.seconds() > 9){
                    yawServo.back();
                    setpathState(8);
                }
                break;
            case 8: // Enable launch servos again
                if (runtime.seconds() > 10){
                    launchServos.enable();
                    setpathState(9);
                }
                break;
        }
    }


    // Setter methods for pathState variables placed at the class level
    void setpathState(int newPathState) {
        this.pathStatePPG = newPathState;
    }



    /**
     * start the AprilTag processor.
     */
}