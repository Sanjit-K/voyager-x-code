package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simple Blue Auto", group = "Test")
public class SimpleBlueAuto extends LinearOpMode {

    // Starting pose (YOUR actual start values)
    private static final Pose START_POSE = new Pose(
            0, 0, Math.toRadians(0)   // fix this angle to match real robot!
    );

    // Where you want to end up to shoot
    private static final Pose SHOOT_POSE = new Pose(
            0, 10, Math.toRadians(0)        // face speaker side
    );

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup Pedro
        Follower follower = Constants.createFollower(hardwareMap);

        // Tell Pedro where we start
        follower.setStartingPose(START_POSE);

        // Build ONE simple path: Start → Shoot
        PathChain goToShoot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(),
                        SHOOT_POSE.getHeading()
                )
                .build();

        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Follow the only path
        follower.followPath(goToShoot);

        // Loop until done
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
