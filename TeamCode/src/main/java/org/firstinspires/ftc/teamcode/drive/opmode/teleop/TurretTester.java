package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp(name = "Turret Tester", group = "Test")
public class TurretTester extends OpMode {
    private Turret turret;
    private Pose robotPose = new Pose(0, 0, 0); // Assume robot is at origin for testing
    private Pose targetPose = new Pose(72, 72, 0); // Target at (72, 72)

    @Override
    public void init() {
        turret = new Turret(hardwareMap, "shooter", "turret", false, false);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            turret.on();
        } else {
            turret.off();
        }

        if (gamepad1.dpad_up) {
            targetPose = new Pose(targetPose.getX(), targetPose.getY() + 1, 0);
        } else if (gamepad1.dpad_down) {
            targetPose = new Pose(targetPose.getX(), targetPose.getY() - 1, 0);
        }

        if (gamepad1.dpad_right) {
            targetPose = new Pose(targetPose.getX() + 1, targetPose.getY(), 0);
        } else if (gamepad1.dpad_left) {
            targetPose = new Pose(targetPose.getX() - 1, targetPose.getY(), 0);
        }

        turret.trackTarget(robotPose, targetPose);

        telemetry.addData("Target X", targetPose.getX());
        telemetry.addData("Target Y", targetPose.getY());
        telemetry.addData("Shooter Power", turret.getShooterPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}

