package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooter;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;


@Configurable
@TeleOp
public class BlueTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72, 72, Math.toRadians(180));
    private TelemetryManager telemetryM;
    private LockMode lockMode;
    private BarIntake intake;
    private KickerServo kickerServo;
    private Shooter shooter;
    private ColorSensor colorSensor;

    private TouchSensor touchSensor;

    //private DistanceSensor distanceSensor;
    private Spindexer spindexer;
    private boolean intakeOn = false;
    private boolean outtakeOn = false;
    private boolean shooterOn = false;
    private char detectedColor = '_';
    private boolean detected = false;
    private int intakeIndex = 0;
    private int shootIndex = 0;

    private int touchSensorTest = 0;

    public char[] filledIndex = {'_', '_', '_'};

    // 3-shot auto volley system
    private boolean threeShotActive = false;
    private ElapsedTime threeShotTimer = new ElapsedTime();

    private final double offset = Math.toRadians(180.0); // Alliance POV offset: 180 = Blue, 0 = Red

    /* Debugging stuff */
    private int detectedCount = 0;
    private String colorLog;
    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) enableAction.run(); else disableAction.run();
        return !currentState;
    }

    public void kickAndClearIndex() {
        int index = spindexer.getShootIndex();   // which chamber is aligned to shooter
        filledIndex[index] = '_';                // clear that chamber
        kickerServo.kick();
    }


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        lockMode = new LockMode(follower);
        intake = new BarIntake(hardwareMap, "intakeMotor", false);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        shooter = new Shooter(hardwareMap, "shooterMotor", false);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");

        //distanceSensor = new DistanceSensor(hardwareMap, "distanceSensor");
        spindexer = new Spindexer(hardwareMap, "spindexer");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        spindexer.setIntakeIndex(intakeIndex);

        follower.setStartingPose(startingPose);
    }

    @Override
    public void start() {follower.startTeleopDrive();}

    @Override
    public void loop() {

        follower.update();

        // Spin outtake
        shooter.on();
        // Field-centric drive with alliance offset, suppressed if locked
        if (gamepad1.left_trigger > 0.5) {
            lockMode.lockPosition();
        }
        else {
            lockMode.unlockPosition();

            // Start teleop drive
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false, // Field centric
                    offset
            );
        }







        if (gamepad1.aWasPressed()){
            intakeOn = toggle(intakeOn,
                    intake::spinIntake,
                    intake::stop
            );
            outtakeOn = false;
        }
        else if (gamepad1.bWasPressed()){
            outtakeOn = toggle(outtakeOn,
                    intake::spinOuttake,
                    intake::stop
            );
            intakeOn = false;
        }

        if (gamepad1.leftBumperWasPressed()) {
            threeShotActive = true;
            threeShotTimer.reset();
        }


        if (threeShotActive) {
//            lockMode.lockPosition(); // lock the robot before shooting
//            aim at function to aim before shooting
            // timing for ball shots
            double t = threeShotTimer.milliseconds();

            // Shooting sequence for ball 1
            if (t > 0   && t < 500)   spindexer.setShootIndex(0);
            if (t > 500 && t < 1000)  kickAndClearIndex();
            if (t > 1000 && t < 1500) kickerServo.normal();

            // Shooting sequence for ball 2
            if (t > 1500 && t < 2000) spindexer.setShootIndex(1);
            if (t > 2000 && t < 2500) kickAndClearIndex();
            if (t > 2500 && t < 3000) kickerServo.normal();

            // Shooting sequence for ball 3
            if (t > 3000 && t < 3500) spindexer.setShootIndex(2);
            if (t > 3500 && t < 4000) kickAndClearIndex();
            if (t > 4000 && t < 4500) kickerServo.normal();

            // turn off shooting sequence
            if (t > 4500) {
                intakeIndex=0;
                spindexer.setIntakeIndex(intakeIndex);
//                lockMode.unlockPosition(); // unlock the robot after shooting
                threeShotActive = false;
            }
        }




        if (gamepad1.xWasPressed()){
            kickAndClearIndex();
        }
        else if (gamepad1.yWasPressed()){
            kickerServo.normal();
        }



        if (gamepad1.rightBumperWasPressed() || touchSensor.isPressed()){
            spindexer.advanceIntake();
        }



        if (gamepad1.dpadUpWasPressed()){
            shootIndex = (shootIndex + 1) % 3;
            spindexer.setShootIndex(shootIndex);
        }

        if (gamepad1.dpadDownWasPressed()) {
            shootIndex = (shootIndex + 2) % 3; // equivalent to (index - 1 + 3) % 3
            spindexer.setShootIndex(shootIndex);
        }

        detectedColor = colorSensor.detection();

        if (detectedColor != '_'){
            colorLog += detectedColor;
            filledIndex[spindexer.getIntakeIndex()] = detectedColor;
        }

//        if(!threeShotActive){
//            if(filledIndex[0] == '_'){
//                spindexer.setIntakeIndex(0);
//            } else if (filledIndex[1] == '_') {
//                spindexer.setIntakeIndex(1);
//            } else if(filledIndex[2] == '_'){
//                spindexer.setIntakeIndex(2);
//            }
//        }

        detected = ((detectedColor == 'P') || (detectedColor == 'G'));

//        if (detected){
//            detectedCount++;
//        }

        //intake 3 balls
        if (gamepad1.leftStickButtonWasPressed()) {
            new Thread(() -> {
                ElapsedTime timer = new ElapsedTime();
                ElapsedTime settle = new ElapsedTime();

                for (int i = 0; i < 3; i++) {

                    spindexer.setIntakeIndex(i);
                    detected = false;
                    timer.reset();

                    while (!detected) {
                        if (timer.milliseconds() >= 10) {
                            detectedColor = colorSensor.detection();
                            detected = (detectedColor == 'P' || detectedColor == 'G');
                            timer.reset();
                        }
                    }

                    settle.reset();
                    while (settle.milliseconds() < 100) {}

                    detected = false;
                }

            }).start();
        }





        telemetryM.debug("Touch Sensor Count", touchSensorTest);
        // Update telemetry
        telemetryM.debug("Deteced Color: ", detectedColor);




        telemetryM.debug("index 0", filledIndex[0]);
        telemetryM.debug("index 1", filledIndex[1]);
        telemetryM.debug("index 2", filledIndex[2]);


        //telemetryM.debug("Detected Object: ", detected);
        telemetryM.debug("Detection Log: ", colorLog);
        telemetryM.debug("Detected Count: ", detectedCount);
        telemetryM.debug("Spindexer Position: ", spindexer.getPosition());
        telemetryM.update(telemetry);
    }
}

