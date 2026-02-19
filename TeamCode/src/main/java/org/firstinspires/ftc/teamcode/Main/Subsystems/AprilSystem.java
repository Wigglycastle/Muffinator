package org.firstinspires.ftc.teamcode.Main.Subsystems;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Main.Utils.DrivePowers;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import com.bylazar.camerastream.PanelsCameraStream;


public class AprilSystem {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.025  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.012  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 1;
    final double MAX_AUTO_STRAFE= 1;
    final double MAX_AUTO_TURN  = 0.75;
    // Camera calibration
    private final int exposureMS = 5;   // Use low exposure time to reduce motion blur
    private final int gain = 1000;
    VisionPortal visionPortal;               // Used to manage the video source.
    AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound     = false;
    double  drive           = 0;
    double  strafe          = 0;
    double  turn            = 0;
    private final Telemetry telemetry;
    public boolean targetDistanceMet = false;
    private  ElapsedTime time = null;
    private boolean timerStart = false;

    public AprilSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal,30);

        setManualExposure(exposureMS, gain);  // Use low exposure time to reduce motion blur
    }

    public DrivePowers CheckForTag(double desiredDistance, double desiredTagID) {
        targetDistanceMet = false;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Not streaming. Check connection.");
            targetFound = false;
            return null; // Exit the method early
        }

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                       telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                   telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - desiredDistance);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // If the error is below a threshold, then the robot is suffiently aligned
                if (rangeError < 0.5 && headingError < 0.5 && yawError < 3 && !timerStart) {
                    time = new ElapsedTime();
                    timerStart = true;
                }
                if (time != null) {
                    if (time.seconds() > 3) {
                        targetDistanceMet = true;
                    }
                }

                // Use the speed and turn "gains" to calculate how the robot should move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            // Calculate wheel powers.
            double frontLeftPower    =  drive - strafe - turn;
            double frontRightPower   =  drive + strafe + turn;
            double backLeftPower     =  drive + strafe - turn;
            double backRightPower    =  drive - strafe + turn;
            // Normalize power values
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
             if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
             }
             // Set power to zero if no target is found
             if (targetFound) {
                 return new DrivePowers(frontLeftPower,backLeftPower,frontRightPower,backRightPower);
             } else {
                 return null;
             }
        }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Define a timeout period
        final long TIMEOUT_MS = 2000; // 2 seconds
        long startTime = System.currentTimeMillis();

        // Wait for the camera to be streaming, but with a timeout and error check
        while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) &&
                (System.currentTimeMillis() - startTime < TIMEOUT_MS)) {

            // Check if the camera has encountered an error
            if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR) {
                telemetry.addData("Camera Error", "Camera failed to initialize!");
                return; // Give up immediately on error
            }
            sleep(20);
        }

        // After the loop, check if we are *actually* streaming
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // Camera is streaming, now we can safely set controls
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50); // Wait for mode to switch
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Manual exposure set");
        } else {
            // We timed out waiting for the camera
            telemetry.addData("Camera Warning", "TIMEOUT waiting for camera to start streaming");
        }
    }

    public int readTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            return -1; // No detections
        }

        // Tags to look for
        Set<Integer> validIds = Set.of(21, 22, 23);

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && validIds.contains(detection.id)) {
                return detection.id; // Return the tag
            }
        }

        return -1; // No matching tag found
    }
}

