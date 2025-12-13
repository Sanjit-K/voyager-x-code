package org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class LockMode {
    private static final double LOCK_DISTANCE = 0.1; // Small oscillation distance

    private boolean wasLocked = false;
    private Path lockForwards;
    private Path lockBackwards;
    private boolean lockForward = true;
    private final Follower follower;

    public LockMode(Follower follower) {
        this.follower = follower;
    }

    public void lockPosition() {
        if (!wasLocked) {
            // Just entered lock: capture pose, activate PIDs, create oscillating paths
            Pose lockPose = follower.getPose();

            // Activate both translational and heading PIDs (like Tuning.java)
            follower.deactivateAllPIDFs();
            follower.activateTranslational();
            follower.activateHeading();

            // Create small oscillating paths to keep PIDs engaged
            // Using diagonal movement to engage both X and Y PIDs
            Pose lockStart = new Pose(lockPose.getX(), lockPose.getY(), lockPose.getHeading());
            Pose lockEnd = new Pose(
                    lockPose.getX() + LOCK_DISTANCE,
                    lockPose.getY() + LOCK_DISTANCE,
                    lockPose.getHeading()
            );

            lockForwards = new Path(new BezierLine(lockStart, lockEnd));
            lockForwards.setConstantHeadingInterpolation(lockPose.getHeading());

            lockBackwards = new Path(new BezierLine(lockEnd, lockStart));
            lockBackwards.setConstantHeadingInterpolation(lockPose.getHeading());

            follower.followPath(lockForwards);
            lockForward = true;
            wasLocked = true;
        } else {
            // While locked: oscillate between forward/backward to keep PIDs fighting pushes
            if (!follower.isBusy()) {
                if (lockForward) {
                    lockForward = false;
                    follower.followPath(lockBackwards);
                } else {
                    lockForward = true;
                    follower.followPath(lockForwards);
                }
            }
        }
    }

    public void unlockPosition() {
        // Exiting lock mode: restore normal teleop drive (only once on transition)
        if (wasLocked) {
            follower.activateAllPIDFs();
            follower.startTeleopDrive();
            wasLocked = false;
        }
    }
}