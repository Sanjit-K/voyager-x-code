package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Improved AprilTag scanner with better detection settings.
 */
public class AprilTagScanner {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private static final double MIN_DECISION_MARGIN = 0; // Minimum confidence threshold

    /**
     * Initialize the AprilTag scanner with optimized settings.
     * @param hardwareMap The hardware map
     * @param webcamName Name of the webcam in the hardware configuration (e.g., "Webcam 1")
     */
    public AprilTagScanner(HardwareMap hardwareMap, String webcamName) {
        // Create the AprilTag processor with optimized settings
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // Standard FTC tags
                .setLensIntrinsics(1385.92f , 1385.92f, 951.982f , 534.084f) // Official C920 calibration (Robert Atkinson, FTC)
                .build();

        // Build the vision portal optimized for C920
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(1920, 1080)) // C920 calibrated for 640x480
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Better performance
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        // Wait for camera to initialize
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // Set manual exposure for better AprilTag detection
    }


    /**
     * Get all currently detected AprilTags that meet the minimum confidence threshold.
     * @return List of detected AprilTags with good confidence
     */
    public List<AprilTagDetection> getDetections() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        // Filter out weak detections
        detections.removeIf(detection -> detection.decisionMargin < MIN_DECISION_MARGIN);
        return detections;
    }

    /**
     * Get all raw detections without filtering.
     * @return List of all detected AprilTags
     */
    public List<AprilTagDetection> getRawDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get a specific AprilTag by ID.
     * @param id The AprilTag ID to search for
     * @return The detection if found with good confidence, null otherwise
     */
    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Check if any AprilTags are visible with good confidence.
     * @return true if at least one tag is detected
     */
    public boolean hasDetections() {
        return !getDetections().isEmpty();
    }

    /**
     * Get the number of currently visible AprilTags with good confidence.
     * @return Number of detected tags
     */
    public int getDetectionCount() {
        return getDetections().size();
    }

    /**
     * Get the camera state for debugging.
     * @return Current camera state
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Get the current FPS for debugging.
     * @return Frames per second
     */
    public double getFps() {
        return visionPortal.getFps();
    }

    /**
     * Get the current FPS for debugging (alias).
     * @return Frames per second
     */
    public double getCameraFPS() {
        return visionPortal.getFps();
    }

    /**
     * Set camera exposure dynamically.
     * @param exposureMS Exposure time in milliseconds (1-100)
     */
    public void setCameraExposure(int exposureMS) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
    }

    /**
     * Set camera gain dynamically.
     * @param gain ISO gain (0-255)
     */
    public void setCameraGain(int gain) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
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
