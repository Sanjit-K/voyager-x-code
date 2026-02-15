package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Turret Tester", group = "Test")
public class TurretTester extends OpMode {
    private Turret turret;
    private Follower follower;
    private final Pose targetPose = new Pose(144, 144, 0); // Fixed target

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Turret constructor: HardwareMap, shooterName, turretName, turretEncoderName, shooterReversed, turretReversed
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);
        follower.setStartingPose(new Pose(7.5, 7.75, 0));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        if (gamepad1.a) {
            turret.on();
        } else {
            turret.off();
        }

        // Track target using current robot pose from follower
        turret.trackTarget(follower.getPose(), targetPose, 0);

        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target X", targetPose.getX());
        telemetry.addData("Target Y", targetPose.getY());
        telemetry.addData("Shooter RPM", turret.getShooterRPM());
        telemetry.addData("Turret Voltage", turret.getTurretVoltage());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
