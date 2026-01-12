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
    private Limelight3A limelight; //any camera here
    private Follower follower;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        follower = Constants.createFollower(hardwareMap);
        limelight.captureSnapshot("on");
        follower.setStartingPose(new Pose(33, 33, 0)); //set your starting pose
    }

    @Override
        public void loop() {

        follower.update();

        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc

        //This uses the aprilTag to relocalize your robot
        follower.setPose(getRobotPoseFromCamera());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    private Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x * 39.3701;
                double y = botpose.getPosition().y * 39.3701;
                return new Pose(x, y, follower.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            }
        }
        return follower.getPose();
    }
}