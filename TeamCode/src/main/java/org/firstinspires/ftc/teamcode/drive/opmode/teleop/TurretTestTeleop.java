package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.turret.LaunchMotor;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.turret.YawServo;

@TeleOp(name = "Turret + 2 Motors (Hold B = Power 1)", group = "Test")
public class TurretTestTeleop extends OpMode {

    // Drive / localization
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // Turret
    private YawServo yawServo;
    private LaunchMotor launchMotor;

    // Two direct motors (e.g., stilts). Change names to match your config.
    private DcMotorEx motorA; // rightRear
    private DcMotorEx motorB; // leftFront

    // Simple state
    private boolean launcherEnabled = true; // toggle with A
    private boolean lastAToggle = false;
    private final boolean frontMount = true; // set false if yaw servo mounted reversed

    @Override
    public void init() {
        // Pedro follower init
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Turret devices (use your config names)
        yawServo    = new YawServo(hardwareMap, follower, "yawServo", frontMount);
        launchMotor = new LaunchMotor(hardwareMap, follower, "rightFront"); // replace with your launcher motor name

        // Direct motors init (replace names if needed)
        motorA = hardwareMap.get(DcMotorEx.class, "rightRear");
        motorB = hardwareMap.get(DcMotorEx.class, "leftFront");

        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Turret + 2 Motors Test: INIT complete");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Drive
        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                false
        );
        follower.update();

        // Launcher enable toggle (A)
        boolean aNow = gamepad1.a;
        if (aNow && !lastAToggle) {
            launcherEnabled = !launcherEnabled;
        }
        lastAToggle = aNow;

        // Turret aim + launcher control
        yawServo.updateDirection(TurretConstants.x0, TurretConstants.y0);
        if (launcherEnabled) {
            launchMotor.updateLaunchMotor();
        }



        // Telemetry
        telemetry.addLine("== Drive/Pose ==");
        telemetry.addData("X (in)", follower.getPose().getX());
        telemetry.addData("Y (in)", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("== Turret ==");
        telemetry.addData("Launcher Enabled", launcherEnabled ? "YES" : "NO");
        telemetry.addData("Target Flywheel RPM", String.format("%.1f", launchMotor.calculateMotorSpeed()));
        telemetry.addData("Aim Target", "(x0,y0)=(" + TurretConstants.x0 + ", " + TurretConstants.y0 + ")");

        telemetry.addLine("== Motors ==");
        telemetry.addData("B held?", gamepad1.b);
        telemetry.addData("motorA power", motorA.getPower());
        telemetry.addData("motorB power", motorB.getPower());

        telemetry.update();
    }

    @Override
    public void stop() {
        motorA.setPower(0.0);
        motorB.setPower(0.0);
        // Optionally stop launcher here if you add a stop() method.
    }
}
