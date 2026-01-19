package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Transfer Test", group = "Test")
public class TransferTest extends OpMode {

    private Follower follower;
    private static final Pose startingPose = new Pose(0,0, Math.toRadians(180));
    private BarIntake barIntake;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private ElapsedTime loopTimer;
    private LynxModule expansionHub;
    private static final double OFFSET = Math.toRadians(180.0);

    @Override
    public void init(){

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor");
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false,false, true);
        loopTimer = new ElapsedTime();
//        spindexer.moveToZero();

        follower.setStartingPose(startingPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET
        );

        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        } else if (gamepad1.xWasPressed()){
            barIntake.stop();
        }

        if (gamepad1.rightBumperWasPressed()){
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()){
            spindexer.retreatIntake();
        }

        if (gamepad1.yWasPressed()){
            spindexer.setIntakeIndex(0);
        }

        if (gamepad1.leftStickButtonWasPressed()){
            spindexer.clearTracking();
        }

        if (gamepad1.dpadRightWasPressed()){
            kickerServo.normal();
        } else if (gamepad1.dpadLeftWasPressed()){
            kickerServo.kick();
        }

        if (gamepad1.dpadUpWasPressed()){
            turret.transferOn();
            turret.on();
        }
        spindexer.update();

        // Telemetry: intake and spindexer status
        double intakePower = barIntake.getPower();
        String intakeMode = intakePower > 0.0 ? "INTAKE" : intakePower < 0.0 ? "OUTTAKE" : "STOPPED";
        telemetry.addData("BarIntake Power", String.format(java.util.Locale.US, "%.3f", intakePower));
        telemetry.addData("Intake Mode", intakeMode);

        telemetry.addData("Spindexer Angle (deg)", String.format(java.util.Locale.US, "%.2f", spindexer.getCalibratedAngle()));
        telemetry.addData("Intake Index", spindexer.getIntakeIndex());
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.addData("Is Full", spindexer.isFull());
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        telemetry.update();
    }
}