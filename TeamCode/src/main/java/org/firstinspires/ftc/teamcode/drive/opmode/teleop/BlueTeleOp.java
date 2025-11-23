package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Shooter;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.DistanceSensor;


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
    private DistanceSensor distanceSensor;
    private Spindexer spindexer;
    private boolean intakeOn = false;
    private boolean outtakeOn = false;
    private boolean shooterOn = false;
    private char detectedColor = '_';
    private boolean detected = false;
    private int intakeIndex = 0;
    private int shootIndex = 0;

    private final double offset = Math.toRadians(180.0); // Alliance POV offset: 180 = Blue, 0 = Red



    /* Debugging stuff */
    private int detectedCount = 0;
    private String colorLog;
    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) enableAction.run(); else disableAction.run();
        return !currentState;
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
        distanceSensor = new DistanceSensor(hardwareMap, "distanceSensor");
        spindexer = new Spindexer(hardwareMap, "spindexer");


        spindexer.setIntakeIndex(intakeIndex);

        follower.setStartingPose(startingPose);

    }

    @Override
    public void start() {follower.startTeleopDrive();}

    @Override
    public void loop() {
        follower.update();

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

        if (gamepad1.xWasPressed()){
            kickerServo.kick();
        }
        else if (gamepad1.yWasPressed()){
            kickerServo.normal();
        }

        if (gamepad1.rightBumperWasPressed()){
            shooterOn = toggle(shooterOn,
                    shooter::on,
                    shooter::off
            );
        }

        if (gamepad1.dpadRightWasPressed()){
            spindexer.advanceIntake();
        }

        if (gamepad1.dpadLeftWasPressed()){
            spindexer.retreatIntake();
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
        detected = distanceSensor.isDetected();

        if (detectedColor != '_'){
            colorLog += detectedColor;
        }

        if (detected){
            detectedCount++;
        }



        // Update telemetry
        telemetryM.debug("Deteced Color: ", detectedColor);
        telemetryM.debug("Detected Object: ", detected);
        telemetryM.debug("Detection Log: ", colorLog);
        telemetryM.debug("Detected Count: ", detectedCount);
        telemetryM.update(telemetry);
    }
}
