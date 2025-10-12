package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Simple AprilTag scanner that detects and provides information about visible AprilTags.
 */
public class AprilTagScanner {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    /**
     * Initialize the AprilTag scanner with a webcam.
     * @param hardwareMap The hardware map
     * @param webcamName Name of the webcam in the hardware configuration (e.g., "Webcam 1")
     */
    public AprilTagScanner(HardwareMap hardwareMap, String webcamName) {
        // Create the AprilTag processor with proper FTC tag library
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Build the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Get all currently detected AprilTags.
     * @return List of detected AprilTags
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get a specific AprilTag by ID.
     * @param id The AprilTag ID to search for
     * @return The detection if found, null otherwise
     */
    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Check if any AprilTags are visible.
     * @return true if at least one tag is detected
     */
    public boolean hasDetections() {
        return !aprilTag.getDetections().isEmpty();
    }

    /**
     * Get the number of currently visible AprilTags.
     * @return Number of detected tags
     */
    public int getDetectionCount() {
        return aprilTag.getDetections().size();
    }

    /**
     * Close the vision portal when done.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Enable or disable the AprilTag processor.
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }
}

