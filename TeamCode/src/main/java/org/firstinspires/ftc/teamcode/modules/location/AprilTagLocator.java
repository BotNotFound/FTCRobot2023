package org.firstinspires.ftc.teamcode.modules.location;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.modules.ModuleBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class AprilTagLocator extends ModuleBase implements Locator {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int tagId;

    public static final String WEBCAM_DEVICE_NAME = "Webcam 1";

    /**
     * Initializes the module and registers it with the specified OpMode
     *
     * @param registrar The OpMode initializing the module
     */
    public AprilTagLocator(@NonNull OpMode registrar, int tagId) {
        super(registrar);
        initAprilTag();
        this.tagId = tagId;
    }

    public AprilTagLocator(@NonNull OpMode registrar) {
        this(registrar, 0);
    }

    public void setTagId(int tagId) {
        this.tagId = tagId;
    }

    public int getTagId() {
        return tagId;
    }

    @Override
    public void cleanupModule() {

    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    parent.hardwareMap.get(WebcamName.class, WEBCAM_DEVICE_NAME), aprilTag);
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    @Override
    public void log() {
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        getTelemetry().addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                getTelemetry().addLine(String.format(Locale.US, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
                getTelemetry().addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                getTelemetry().addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                getTelemetry().addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                getTelemetry().addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", detection.id));
                getTelemetry().addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        getTelemetry().addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        getTelemetry().addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        getTelemetry().addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    @Override
    public LocalizedMovement getLocation() throws LocatorException {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagId) {
                AprilTagPoseFtc pose = detection.ftcPose;
                return new LocalizedMovement(
                        pose.x,
                        pose.y,
                        pose.yaw,
                        this
                );
            }
        }
        throw new LocatorException(this, "Target April Tag not detected!");
    }

    @Override
    public LocatorKind getKind() {
        return LocatorKind.OBJECT_RELATIVE;
    }

    @Override
    public Movement getFieldSize() {
        return new Movement(
                144,
                144,
                360
        );
    }

    @Override
    public boolean isActive() {
        if (tagId < 1) {
            return false;
        }
        return Locator.super.isActive();
    }
}   // end class
