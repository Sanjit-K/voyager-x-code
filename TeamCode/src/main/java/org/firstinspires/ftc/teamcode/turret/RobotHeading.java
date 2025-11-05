package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;

public class RobotHeading {
    private final Follower follower;

    private final double goalX = 0;
    private final double goalY = 144;

    public RobotHeading(Follower follower) {
        this.follower = follower;
    }

    public void aimAtGoal() {
        // Current robot pose
        Pose robotPose = follower.getPose();

        // Calculate heading to goal
        double dx = goalX - robotPose.getX() + 2; // slight offset to avoid zero-length path
        double dy = goalY - robotPose.getY() + 2;
        double desiredHeading = Math.atan2(dy, dx) + Math.PI;

        // Make final position move slightly to the left to ensure the path has motion
        Pose adjusted = new Pose(robotPose.getX() + 2, robotPose.getY() + 2, desiredHeading);


        // Path to change heading
        PathChain faceGoalPath = follower.pathBuilder()
                .addPath(new BezierLine(robotPose, adjusted))
                .setLinearHeadingInterpolation(robotPose.getHeading(), -desiredHeading, 0.2)
                .build();


        // follow the path
        follower.followPath(faceGoalPath);
    }

    public void shootAtGoal() {

    }
}