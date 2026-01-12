package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp
public class LocalizationTestTeleop extends OpMode {
    private Limelight3A limelight;
    private Follower follower;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33, 33, 0)); // Pedro coords (in)

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);   // make sure pipeline 0 is APRILTAG
        limelight.start();
    }

    @Override
    public void loop() {
        Pose camPose = getRobotPoseFromCamera();
        if (camPose != null) {
            follower.setPose(camPose);
        }

        follower.update();

        telemetry.addData("Has Cam Pose", camPose != null);
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        // Require at least one tag
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        // Limelight reports meters -> convert to inches
        double xIn = botpose.getPosition().x * 39.3701;
        double yIn = botpose.getPosition().y * 39.3701;

        // IMPORTANT: get yaw from Limelight pose, not follower
        // Depending on SDK version, you may need AngleUnit.DEGREES/RADIANS.
        // If this line doesn’t compile, tell me your Pose3D methods and I’ll adjust.
        double headingRad = Math.toRadians(botpose.getOrientation().getYaw());

        Pose ftcPose = new Pose(xIn, yIn, headingRad, FTCCoordinates.INSTANCE);

        // Convert into Pedro coordinate system
        return ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
