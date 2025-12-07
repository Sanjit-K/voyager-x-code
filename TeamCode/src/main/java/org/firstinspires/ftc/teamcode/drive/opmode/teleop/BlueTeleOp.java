package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.intake.IntakeFlap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooter;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


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
    private IntakeFlap intakeFlap;
    private DigitalChannel distanceSensor;



    private Spindexer spindexer;
    private volatile boolean intakeOn = false;
    private volatile boolean outtakeOn = false;
    private char detectedColor = '_';
    private boolean detected = false;
    private int intakeIndex = 0;
    private boolean rightPos = false;


    // 3-shot auto volley system
    private boolean threeShotActive = false;
    private final ElapsedTime threeShotTimer = new ElapsedTime();
    private final ElapsedTime detectedTimer = new ElapsedTime();
    private static final int SHOOT_DELAY = 350;
    private static final int KICK_DELAY = 250;
    private static final int NORMAL_DELAY = 200;
    private static final int FULL_CYCLE = SHOOT_DELAY + KICK_DELAY + NORMAL_DELAY;

    private final int DETECTED_DELAY = 300;

    private final double offset = Math.toRadians(180.0); // Alliance POV offset: 180 = Blue, 0 = Red

    /* Debugging stuff */
    private int detectedCount = 0;
    private String colorLog;


    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) enableAction.run(); else disableAction.run();
        return !currentState;
    }

    public void kickAndClearIndex(int index) {
        spindexer.setColorAtPos('_', index);
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
        spindexer = new Spindexer(hardwareMap, "spindexer");
        intakeFlap = new IntakeFlap(hardwareMap, "intakeFlapServo");

        distanceSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        distanceSensor.setMode(DigitalChannel.Mode.INPUT);

        spindexer.setIntakeIndex(intakeIndex);
        intakeFlap.off();
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
        else{

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

        detected = distanceSensor.getState();



        if (gamepad1.aWasPressed()){
            intakeOn = toggle(intakeOn,
                    intake::spinIntake,
                    intake::stop
            );
            outtakeOn = false;
        }
        else if (gamepad1.bWasPressed()){
            new Thread(() -> {
                outtakeOn = toggle(outtakeOn,
                        intake::spinOuttake,
                        intake::stop
                );
                intakeOn = false;
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {}
                intakeOn = toggle(intakeOn,
                        intake::spinIntake,
                        intake::stop
                );
                outtakeOn = false;
            }).start();
        }


        if (gamepad1.leftBumperWasPressed()) {
            threeShotActive = true;
            intakeFlap.on();
            threeShotTimer.reset();
        }

        int fd = 0;
        if (threeShotActive) {
//            aim at function to aim before shooting
            // timing for ball shots
            double t = threeShotTimer.milliseconds();
            intake.spinOuttake();

            fd = rightPos ? SHOOT_DELAY : 0;
            // Shooting sequence for ball 1
            if (t > 0 && t < SHOOT_DELAY - fd) spindexer.setShootIndex(2);
            if (t > SHOOT_DELAY - fd && t < SHOOT_DELAY + KICK_DELAY - fd)  kickAndClearIndex(2);
            if (t > SHOOT_DELAY + KICK_DELAY - fd && t < FULL_CYCLE - fd) kickerServo.normal();

            // Shooting sequence for ball 2
            if (t > FULL_CYCLE - fd && t < FULL_CYCLE + SHOOT_DELAY - fd) spindexer.setShootIndex(0);
            if (t > FULL_CYCLE + SHOOT_DELAY - fd && t < 2*FULL_CYCLE - NORMAL_DELAY - fd) kickAndClearIndex(0);
            if (t > 2*FULL_CYCLE - NORMAL_DELAY - fd && t < 2*FULL_CYCLE - fd) kickerServo.normal();

            // Shooting sequence for ball 3
            if (t > 2*FULL_CYCLE - fd && t < 2*FULL_CYCLE + SHOOT_DELAY - fd) spindexer.setShootIndex(1);
            if (t > 2*FULL_CYCLE + SHOOT_DELAY - fd&& t < 3*FULL_CYCLE - NORMAL_DELAY - fd) kickAndClearIndex(1);
            if (t > 3*FULL_CYCLE - NORMAL_DELAY - fd&& t < 3*FULL_CYCLE - fd) kickerServo.normal();

            // turn off shooting sequence
            if (t > 3*FULL_CYCLE - fd) {
                intakeIndex = 0;
                spindexer.setIntakeIndex(intakeIndex);
                intake.spinIntake();
                intakeFlap.off();
                threeShotActive = false;
                rightPos = false;
            }
        }




        if (gamepad1.xWasPressed()){
            kickerServo.kick();
        }
        else if (gamepad1.yWasPressed()){
            kickerServo.normal();
        }



        detectedColor = colorSensor.detection();

        if ((detected && !spindexer.isFull()  &&  detectedTimer.milliseconds() > DETECTED_DELAY) && !threeShotActive){
            spindexer.setColorAtPos(detectedColor);
            if (!spindexer.isFull()){
                spindexer.advanceIntake();
                rightPos = false;
            }
            else{
                intakeFlap.on();
                spindexer.setShootIndex(2);
                rightPos = true;
            }
            detectedCount++;
            detectedTimer.reset();
        }

        else if (gamepad1.rightBumperWasPressed()){
            spindexer.setColorAtPos(detectedColor);
            spindexer.advanceIntake();
            intakeFlap.off();
            rightPos = false;
            detectedCount++;
            detectedTimer.reset();
        }



        // Update telemetry
        telemetryM.debug("Detected Color: ", detectedColor);


        //telemetryM.debug("Detected Object: ", detected);
        char[] filled = spindexer.getFilled();
        telemetryM.debug("Detection Log: ", colorLog);
        telemetryM.debug("Detected Count: ", detectedCount);
        telemetryM.debug("Spindexer Position: ", spindexer.getPosition());
        telemetryM.debug("index 0", filled[0]);
        telemetryM.debug("index 1", filled[1]);
        telemetryM.debug("index 2", filled[2]);
        telemetryM.debug("fd", fd);

        telemetryM.update(telemetry);
    }
}

