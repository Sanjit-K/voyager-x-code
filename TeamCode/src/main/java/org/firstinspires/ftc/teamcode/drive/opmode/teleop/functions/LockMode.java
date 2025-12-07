package org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class LockMode {

    private boolean wasLocked = false;

    private final Follower follower;

    public LockMode(Follower follower) {
        this.follower = follower;
    }

    public void lockPosition() {
        if (!wasLocked) {
            Pose lockPose = follower.getPose();

            follower.holdPoint(lockPose);
            wasLocked = true;

        }
    }

    public void unlockPosition() {
        // Exiting lock mode: restore normal teleop drive (only once on transition)
        if (wasLocked) {
            follower.startTeleopDrive();
            wasLocked = false;
        }
    }
}
