package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@TeleOp
public class TurretLimelight extends OpMode {
    private Limelight3A limelight; //any camera here
    private Follower follower;
    Turret turret;

    /**
     * Simple open-loop target angle for the servo. We no longer have a turret angle getter,
     * so we track the commanded angle locally.
     */
    private double targetAngleDeg = 180.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1); // Switch to pipeline number 1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); //set your starting pose
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false);

        // Start centered/backwards by convention
        targetAngleDeg = 180.0;
        turret.goToPosition(targetAngleDeg);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)

            // Adjust our commanded target angle. Clamp to the same range enforced in trackTarget().
            targetAngleDeg = Math.max(80.0, Math.min(280.0, targetAngleDeg + tx));
            turret.goToPosition(targetAngleDeg);

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Angle (cmd)", targetAngleDeg);
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.addData("Target Angle (cmd)", targetAngleDeg);
        }
        telemetry.addData("Turret Voltage", turret.getTurretVoltage());
        telemetry.update();
    }
}
