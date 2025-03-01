package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PixelGroupDetectionProcessor;
// docs: https://javadoc.io/doc/org.firstinspires.ftc/RobotCore/latest/org/firstinspires/ftc/robotcore/internal/camera/delegating/CachingExposureControl.html
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.util.SpeedCoefficients;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.cvpipelines.PropDetectionPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class CVSubsystem extends SubsystemBase {
    private OpenCvCamera camera;
    private DriveSubsystem drive;

    public final int SAMPLE_COUNT = 69;
    public final long SAMPLE_WAIT_MILLISECONDS = 22;
    public final int LOCATION_LEFT   =  0;
    public final int LOCATION_CENTER =  1;
    public final int LOCATION_RIGHT  =  2;
    public final int NO_LOCATION     = -1;
    private Telemetry telemetry;
    final private ElapsedTime runtime = new ElapsedTime();

    public final double NO_ROTATIONAL_OFFSET = -50000.0;
    public final double NO_DISTANCE = -50000.0;
    public final double ERROR   =  3.0;
    public final double ERROR_ALIGNMENT = 4;

    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public final WebcamName cameraName1, cameraName2;
//    public final ExposureControl exposureControl;
    private SwitchableCameraName switchableCamera;
    private PropDetectionPipeline propProcessor;
    private PixelGroupDetectionProcessor pixelGroupProcessor;
    private boolean isRedTeam;
    private LinearOpMode opMode;
    private static final double[][] APRIL_TAG_LOCATIONS = new double[][]{
            {1.25, 0.5}, {1.5, 0.5}, {1.75, 0.5},
            {4.25, 0.5}, {4.5, 0.5}, {4.75, 0.5},
            {4.75, 6}  , {4.5, 6}  ,
            {1.25, 6}  , {1.5, 6}
    };

    // Example fields to store analysis results
    private String detectedColor = "";
    private List<Scalar> leftTopHSV = new ArrayList<>();
    private List<Scalar> centerTopHSV = new ArrayList<>();
    private List<Scalar> rightTopHSV = new ArrayList<>();

    // identifyColorRangesInLocations implementation
    // Assume this method updates the above fields with the latest analysis results

    // Public getters for oranjejuce to use
    public String getDetectedColor() { return detectedColor; }
    public List<Scalar> getLeftTopHSV() { return leftTopHSV; }
    public List<Scalar> getCenterTopHSV() { return centerTopHSV; }
    public List<Scalar> getRightTopHSV() { return rightTopHSV; }

    public CVSubsystem(WebcamName cameraName1, WebcamName cameraName2, DriveSubsystem drive, Telemetry telemetry, boolean isRedTeam, LinearOpMode opMode) {
        this(null, cameraName1, cameraName2,drive,telemetry,isRedTeam,opMode);
    }

    public CVSubsystem(OpenCvCamera camera, WebcamName cameraName1, WebcamName cameraName2, DriveSubsystem drive, Telemetry telemetry, boolean isRedTeam, LinearOpMode opMode) {
//        tdp = new TapeDetectionPipeline();
//        camera.setPipeline(tdp);

        this.camera = camera;

        this.drive = drive;
        this.telemetry = telemetry;
        this.cameraName1 = cameraName1;
        this.cameraName2 = cameraName2;
        switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(this.cameraName2, this.cameraName1);
        this.isRedTeam = isRedTeam;
        this.opMode = opMode;
        runtime.reset();
//        exposureControl = OpenCvSwitchableWebcam.getInstance().createWebcam(frontCamName);

        // create AprilTagProcessor and VisionPortal
        initAprilTag();


//        pixelPipeline = new PixelDetectionPipeline();
//        propPipeline = new PropDetectionPipeline(isRedTeam, telemetry);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                initAprilTag();
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
    }

    private void initAprilTag() {
        //camera exposure control
//        exposureControl.setExposure(exposureControl.getMinExposure(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS);
//        exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
        // Create the AprilTag processor
        propProcessor = new PropDetectionPipeline(isRedTeam);
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();
        pixelGroupProcessor = new PixelGroupDetectionProcessor();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(cameraName2, cameraName1);

        builder.setCamera(switchableCamera);

        builder.setCameraResolution(new Size(640, 480)); // android.util

        // TODO: DISABLE PROPPROCESSOR FOR TELEOP
        builder.addProcessors(aprilTag, propProcessor);
//        builder.setAutoStopLiveView(false);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.setProcessorEnabled(propProcessor, true);
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);
//        switchCamera(cameraName2);
        autoExposure();
    }

    public void autoExposure() {
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        exposureControl.setMode(ExposureControl.Mode.Manual);
//        // tune duration at comp, higher for brighter view (ie. in darker environments)
//        exposureControl.setExposure(10, TimeUnit.MILLISECONDS);
    }

    public void decShutter() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
    }

    public void disablePropProcessor() {
        visionPortal.setProcessorEnabled(propProcessor, false);
    }

    public void close() {
        visionPortal.close();
    }

    public void switchCamera(WebcamName cameraName) {
        visionPortal.setActiveCamera(cameraName);
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return whether the AprilTag is left, center, or right in the camera view
     */
    public int getAprilTagLocation(int tagID) {
//        visionPortal.resumeStreaming();
        //this is old for testing apriltag team prop thingy

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                if (detection.id == tagID) {
                    if (detection.ftcPose.z < -ERROR) return LOCATION_LEFT;
                    else if (detection.ftcPose.z >  ERROR) return LOCATION_RIGHT;
                    else return LOCATION_CENTER;
                }
            }
            return -2;
        }

        return NO_LOCATION;
    }

    /**
     * GROUP 3
     * @param tagID the id of the AprilTag from the 36h11 family
     * @return a double representing the amount the robot should turn to be "parallel" to the AprilTag
     */
    public double getAprilTagRotationalOffset(int tagID) { // return yaw
//        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double rotationalOffset = NO_ROTATIONAL_OFFSET;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    rotationalOffset = detection.ftcPose.yaw;
                }
            }
        }

        return rotationalOffset;
    }

    public double getAprilTagDistance(Integer... tagIDs) {
//        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = Double.MAX_VALUE;
        Set<Integer> tagSearchSet = new HashSet<>(Arrays.asList(tagIDs));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && tagSearchSet.contains(detection.id)) {
                double d = detection.ftcPose.range;
                if (d < tagDistance)
                    tagDistance = d;
            }
        }
//        visionPortal.stopStreaming();

        return tagDistance;
    }

    public double getAprilTagDistance(int tagID) {
//        visionPortal.resumeStreaming();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = NO_DISTANCE;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    tagDistance = detection.ftcPose.range;
                }
            }
        }
//        visionPortal.stopStreaming();

        return tagDistance;
    }

    /**
     * GROUP 2
     * @return whether the AprilTag is left, center, or right in the camera view based on a mode of samples
     */
    public int getPropLocation() {
        int propLocation = -1;
        ArrayList<Integer> samples = new ArrayList<>();
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            if (!opMode.opModeIsActive()) break;
            propLocation = propProcessor.getPosition();
            if (-1 < propLocation && propLocation < 3) samples.add(propLocation);
            sleepFor(SAMPLE_WAIT_MILLISECONDS);
        }
        // parsing early misreads
        int[] locations = new int[3];
        for (int i : samples) locations[i]++;
        int max = 0;
        for (int i = 0; i < 3; i++) {
            if (locations[i] > max) {
                max = locations[i];
                propLocation = i;
            }
        }
        telemetry.addData("",samples);
        telemetry.addData("loc: ",propLocation);
        telemetry.addData("locs0: ", locations[0]);
        telemetry.addData("locs1: ", locations[1]);
        telemetry.addData("locs2: ", locations[2]);
        telemetry.update();
        return propLocation;
    }

    // New function to identify color ranges in locations
    public void identifyColorRangesInLocations() {
        // Corrected and valid HSV color ranges
        HashMap<String, Scalar[]> colorRanges = new HashMap<>();
        colorRanges.put("Red", new Scalar[]{new Scalar(0, 100, 100), new Scalar(10, 255, 255)}); // Lower range for Red
        colorRanges.put("RedUpper", new Scalar[]{new Scalar(160, 100, 100), new Scalar(179, 255, 255)}); // Upper range for Red
        colorRanges.put("Blue", new Scalar[]{new Scalar(100, 150, 150), new Scalar(140, 255, 255)}); // Range for Blue

        Mat frame = new Mat(); // Placeholder for captured frame
        Mat hsvFrame = new Mat();

        // Assuming a continuous or periodic capture process, not a synchronous loop
        // Convert frame to HSV color space once per frame capture
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Define ROIs for left, center, right locations based on frame dimensions
        int frameWidth = hsvFrame.width();
        int frameHeight = hsvFrame.height();
        Rect leftROI = new Rect(0, 0, frameWidth / 3, frameHeight);
        Rect centerROI = new Rect(frameWidth / 3, 0, frameWidth / 3, frameHeight);
        Rect rightROI = new Rect(2 * frameWidth / 3, 0, frameWidth / 3, frameHeight);

        // Analyze each ROI for color ranges using the new method
        analyzeColorPresenceAndHSVFrequency(hsvFrame.submat(leftROI), "Left", colorRanges);
        analyzeColorPresenceAndHSVFrequency(hsvFrame.submat(centerROI), "Center", colorRanges);
        analyzeColorPresenceAndHSVFrequency(hsvFrame.submat(rightROI), "Right", colorRanges);
    }

    private void analyzeColorPresenceAndHSVFrequency(Mat roi, String location, HashMap<String, Scalar[]> colorRanges) {
        HashMap<Scalar, Integer> hsvFrequency = new HashMap<>();
        boolean colorDetected = false;

        for (Map.Entry<String, Scalar[]> entry : colorRanges.entrySet()) {
            Mat mask = new Mat();
            Core.inRange(roi, entry.getValue()[0], entry.getValue()[1], mask);

            if (Core.countNonZero(mask) > 0) {
                colorDetected = true;
                System.out.println(location + " contains " + entry.getKey());
            }

            // Assuming you have a method to convert mask to HSV and then count frequencies
            // This part is pseudocode and needs actual implementation
            for (int i = 0; i < mask.rows(); i++) {
                for (int j = 0; j < mask.cols(); j++) {
                    if (mask.get(i, j)[0] != 0) { // Check if the pixel is within the color range
                        double[] hsvPixel = roi.get(i, j);
                        Scalar hsv = new Scalar(hsvPixel);
                        hsvFrequency.putIfAbsent(hsv, 0);
                        hsvFrequency.put(hsv, hsvFrequency.get(hsv) + 1);
                    }
                }
            }

            mask.release();
        }

        if (colorDetected) {
            // Sort by frequency and pick top 3
            List<Map.Entry<Scalar, Integer>> list = new ArrayList<>(hsvFrequency.entrySet());
            list.sort(Map.Entry.comparingByValue(Comparator.reverseOrder()));

            System.out.println(location + " Top 3 HSV Values:");
            list.stream().limit(3).forEach(entry -> {
                System.out.println(entry.getKey() + " => " + entry.getValue());
            });
        } else {
            System.out.println(location + " contains no detected colors from the given ranges.");
        }
    }

    public int getPixelHorizontalOffset() {
//        visionPortal.resumeStreaming();
        return 0;  // TODO
    }

    public int getWhitePixelHorizontalOffset() {
        return 0;  // TODO
    }

    public double getWhitePixelDiameterPx() {
        return 0; // TODO: WRITE
    }

    public int getCameraWidth() {
        return pixelGroupProcessor.getCameraWidth();
    }

    public double getAprilTagHorizontalOffset(int tagID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double tagDistance = NO_DISTANCE;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tagID) {
                    tagDistance = detection.ftcPose.z;
                    return tagDistance;
                }
            }
        }
        return tagDistance;
    }

    private double[] lastKnownPos = POSITION_UNKNOWN;
    public static final double[] POSITION_UNKNOWN = new double[]{-1, -1, -1};
    /**
     * returns the x,y,rotation position of the center of the robot based on apriltag positions in tiles. blue-backdrop corner is considered 0,0.
     * facing backdrop is zero rotation, right is more rotation
     * offset params are positive towards the direction the camera is facing. the camera is assumed to be facing either "robot-forward" or "robot-backward"
     */
    public double[] getPosition(double cameraCenterOffsetX, double cameraCenterOffsetY) { // of apriltags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() == 0) return lastKnownPos;
        ArrayList<Double> xEst   = new ArrayList<>(); //estimated x positions based on aprilTag
        ArrayList<Double> yEst   = new ArrayList<>();
        ArrayList<Double> rotEst = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.ftcPose != null) {
                double rotOff = detection.ftcPose.yaw;
                // 24 inches per tile
                double radians = Math.toRadians(detection.ftcPose.bearing - detection.ftcPose.yaw);
                double xOff   = (Math.cos(radians) * detection.ftcPose.range) / 24 - cameraCenterOffsetX;
                double yOff   = (Math.sin(radians) * detection.ftcPose.range) / 24 - cameraCenterOffsetY;
                if (detection.id < 7) {
                    xEst.add(APRIL_TAG_LOCATIONS[detection.id-1][0] + yOff);
                    yEst.add(APRIL_TAG_LOCATIONS[detection.id-1][1] + xOff);
                    rotEst.add((rotOff + 360) % 360);
                } else {
                    xEst.add(APRIL_TAG_LOCATIONS[detection.id-1][0] - yOff);
                    yEst.add(APRIL_TAG_LOCATIONS[detection.id-1][1] - xOff);
                    rotEst.add((rotOff + 540) % 360);
                }
            }
        }
        Collections.sort(xEst);
        Collections.sort(yEst);
        Collections.sort(rotEst);
        double xMed = 0, yMed = 0, rotMed = 0;
        if (xEst.size() > 0) xMed = xEst.get(xEst.size() / 2);
        if (yEst.size() > 0) yMed = yEst.get(yEst.size() / 2);
        if (rotEst.size() > 0) rotMed = rotEst.get(rotEst.size() / 2);
        lastKnownPos = new double[]{xMed, yMed, rotMed};
        //xMed, yMed is in term of number of tiles
        return lastKnownPos;
    }

    public double angleDifference(double angle1, double angle2)
    {
        double d=angle2-angle1;
        if (d > 180) d-=360;
        if (d < 180) d+=360;
        return d;
    }

    public static class LocalizationState {
        public boolean unchanged;
        public double xPower;
        public double yPower;
        public double aPower;
        public LocalizationState(boolean unchanged, double xPower, double yPower, double aPower) {
            this.unchanged = unchanged;
            this.xPower = xPower;
            this.yPower = yPower;
            this.aPower = aPower;
        }
    }

    public static class Position {
        public double x;
        public double range;
        public double turn;
        public Position(double x, double range, double turn) {
            this.x = x;
            this.range = range;
            this.turn = turn;
        }
    }

    public AprilTagPoseFtc getPosToAprilTag(int tagid) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == tagid) {
                return detection.ftcPose;
            }
        }
        return null;
    }

    @Deprecated
    public LocalizationState faceTag(int id, double xOffsetTarget, double yOffsetTarget, double aOffsetTarget, LocalizationState lastState, double _X_THRESHOLD, double _Y_THRESHOLD, double _A_THRESHOLD, double _X_SPEED_COEF, double _Y_SPEED_COEF, double _A_SPEED_COEF, double _X_SLOW_FACTOR, double _Y_SLOW_FACTOR, double _A_SLOW_FACTOR, double _X_SPEED_LIMIT, double _Y_SPEED_LIMIT, double _A_SPEED_LIMIT) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean unchanged = false;
        boolean detected = false;
        double xPower = 0, yPower = 0, aPower = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                unchanged = true;
                detected = true;
                // negative x offset is more left target
                double xDiff = (detection.ftcPose.range*Math.sin(Math.toRadians(detection.ftcPose.yaw))) + xOffsetTarget;  // robot left of target is positive, right is negative
                // positive y offset is more "in" target
                double yDiff = (detection.ftcPose.range*Math.cos(Math.toRadians(detection.ftcPose.yaw))) - yOffsetTarget;  // positive means too far, negative means too close
                // positive angle offset is more counterclockwise target
                double aDiff = detection.ftcPose.yaw + aOffsetTarget;  // left turn of robot is more negative, right turn is more positive
                if (Math.abs(xDiff) > _X_THRESHOLD) {
                    xPower = Math.signum(xDiff)*Math.min(Math.pow(Math.abs(xDiff), _X_SLOW_FACTOR) * _X_SPEED_COEF, _X_SPEED_LIMIT);
                    unchanged = false;
                }
                if (Math.abs(yDiff) > _Y_THRESHOLD) {
                    yPower = -Math.signum(yDiff)*Math.min(Math.pow(Math.abs(yDiff), _Y_SLOW_FACTOR) * _Y_SPEED_COEF, _Y_SPEED_LIMIT);
                    unchanged = false;
                }
                if (Math.abs(aDiff) > _A_THRESHOLD) {
                    aPower = -Math.signum(aDiff)*Math.min(Math.pow(Math.abs(aDiff), _A_SLOW_FACTOR) * _A_SPEED_COEF, _A_SPEED_LIMIT);
                    unchanged = false;
                }
            }
        }
        if (!detected || unchanged) {
            drive.driveRobotCentric(-xPower, -yPower, -aPower);
        } else {
            drive.driveRobotCentric(xPower, yPower, aPower);
        }
        return new LocalizationState(unchanged, xPower, yPower, aPower);
    }

    @Deprecated
    public boolean goToPosition(double cameraCenterOffsetX, double cameraCenterOffsetY, double[] targetPos, double speedFactor, double slowFactor, double angleFactor, double xyOffsetThreshold, double angleOffsetThreshold){
        double[] pos = getPosition(cameraCenterOffsetX, cameraCenterOffsetY);
        double xOffset = pos[0]-targetPos[0];
        double yOffset = pos[1]-targetPos[1];
        double angleOffset = angleDifference(pos[2],targetPos[2]);
        boolean good = true;
        double xd=0, yd=0, ad=0;
        double speedLimit = 0.5;
        if (Math.abs(xOffset) > xyOffsetThreshold) {
            xd = Math.signum(xOffset)*Math.min(Math.pow(Math.abs(xOffset), slowFactor)*Math.sin(Math.toRadians(angleOffset))*speedFactor, speedLimit) * SpeedCoefficients.getAutonomousDriveSpeed();
            good = false;
        }
        if (Math.abs(yOffset) > xyOffsetThreshold) {
            yd = Math.signum(yOffset)*Math.min(Math.pow(Math.abs(yOffset), slowFactor)*Math.cos(Math.toRadians(angleOffset))*speedFactor, speedLimit) * SpeedCoefficients.getAutonomousDriveSpeed();
            good = false;
        }
        if (Math.abs(angleOffset) > angleOffsetThreshold) {
            ad = Math.signum(angleOffset)*Math.min(Math.pow(Math.abs(angleOffset*angleFactor), slowFactor) * speedFactor, speedLimit * angleFactor) * SpeedCoefficients.getAutonomousTurnSpeed();
            good = false;
        }
        telemetry.addData("xd", xd);
        telemetry.addData("yd", yd);
        telemetry.addData("ad", ad);
        telemetry.addData("xdiff", xOffset);
        telemetry.addData("ydiff", yOffset);
        telemetry.addData("adiff", angleOffset);
        drive.driveRobotCentric(xd, -yd, ad);
        return good;
    }

    public void moveToPixel() {
        double ERROR_THRESHOLD = 50;

        int pixelOffset = getPixelHorizontalOffset();
        while (Math.abs(pixelOffset) > ERROR_THRESHOLD) {
            if (pixelOffset < 0){
                drive.driveRobotCentric(1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }else{
                drive.driveRobotCentric(-1 * SpeedCoefficients.getStrafeSpeed(), 0, 0);
            }
            pixelOffset = getPixelHorizontalOffset();
        }
    }

    public void moveToWhitePixel() {
        // TODO: RECODE
//        int width = getCameraWidth();
//        double HORIZ_THRESHOLD = width / 11.1;
//        double DIAM_THRESHOLD = width / 4.0;
//        int pixelOffset = getWhitePixelHorizontalOffset();
//        double pixelDiam = getWhitePixelDiameterPx();
//        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDiam) < DIAM_THRESHOLD && opMode.opModeIsActive()) {
//            drive.driveRobotCentric(Math.max(DIAM_THRESHOLD, pixelDiam) / DIAM_THRESHOLD, Math.min(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, 0);
//            pixelOffset = getWhitePixelHorizontalOffset();
//            pixelDiam = getWhitePixelDiameterPx();
//        }
    }

    public Point getPixelsCenter() {
        visionPortal.setProcessorEnabled(propProcessor, true);
        // do-while used bc idk if we will immediately get results, we'll have to
        // wait a frame interval before pixels center returns actual results
        // do-while also cancels bad results
        // maybe make this do while (and future ones) async?
        Point c;
        do {
            c = pixelGroupProcessor.getPixelsCenter();
        }
        while (opMode.opModeIsActive() && !c.equals(new Point(0, 0)));
        visionPortal.setProcessorEnabled(propProcessor, false);
        return c;
    }

    // TODO: CONSIDER REMOVING IN FAVOR OF APRILTAG ALIGNMENT AND JUST HUMAN PLAYER PLACING PIXELS IN SAME SPOTS
    public void moveToPixels() {
//        visionPortal.setProcessorEnabled(propProcessor, true);
//        int width = getCameraWidth();
//        double HORIZ_THRESHOLD = width / 11.1;
//        double DIST_THRESHOLD = width / 4.0;
//        // same comments regarding do-while as in getPixelsCenter()
//        Point p;
//        do {
//            p = pixelGroupProcessor.getPixelsCenter();
//        }
//        while (opMode.opModeIsActive() && !p.equals(new Point(0, 0)));
//        double pixelOffset = p.x;
//        double pixelDist = p.y;
//        // TODO: NEED MUCH TUNING
//        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDist) < DIST_THRESHOLD && opMode.opModeIsActive()) {
//            drive.driveRobotCentric(Math.max(DIST_THRESHOLD, pixelDist) / DIST_THRESHOLD, Math.min(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, 0);
//            pixelOffset = pixelGroupProcessor.getPixelsCenter().x;
//            pixelDist = pixelGroupProcessor.getPixelsCenter().y;
//        }
//        visionPortal.setProcessorEnabled(propProcessor, false);
    }

    public void correctPositionAprilTag(int targetx, int targety) {
        double[] currentPos = getPosition(99, 99);
        drive.driveRobotCentric(currentPos[0], currentPos[1], currentPos[2]);
    }

    public void moveToAprilTag(int tagID) {
        int width = getCameraWidth();
        double HORIZ_THRESHOLD = width / 11.1;
        double DIST_THRESHOLD = 22;
        double pixelOffset = getAprilTagHorizontalOffset(tagID);
        double pixelDist = getAprilTagDistance(tagID);
        while (Math.abs(pixelOffset) > HORIZ_THRESHOLD || Math.abs(pixelDist) > DIST_THRESHOLD) {
            drive.driveRobotCentric(Math.max(HORIZ_THRESHOLD, pixelOffset) / HORIZ_THRESHOLD, Math.max(DIST_THRESHOLD, pixelDist) / DIST_THRESHOLD, 0);
            pixelOffset = getWhitePixelHorizontalOffset();
            pixelDist = getWhitePixelDiameterPx();
        }
    }

    /**
     * aligns such that a ray representing a robot's camera direction is (anti-)parallel to a ray extending "outwards" from the center of a given AprilTag
     * @param tagID the id of the AprilTag from the 36h11 family to align with
     */
    public void alignParallelWithAprilTag(int tagID) {
        double rotOff = getAprilTagRotationalOffset(tagID); //counter clockwise yaw is positive
        telemetry.addData("err", Math.abs(rotOff));
        if (rotOff == NO_ROTATIONAL_OFFSET) {
            // maybe do something
        } else {
            while (Math.abs(getAprilTagRotationalOffset(tagID)) > ERROR_ALIGNMENT && opMode.opModeIsActive()) {
                //assume that we don't need to optimize getAprilTagRotationalOffset(tagID) since it runs anyway
                drive.driveFieldCentric(0, 0, rotOff * 0.1 * SpeedCoefficients.getTurnSpeed()); // times some scaling factor (temporarily at 1)
            }
            drive.stopController();
//            while (Math.abs(rotOff) > ERROR_ALIGNMENT && opMode.opModeIsActive()) {
//                drive.driveByEncoder(0, 0, 0.2, (double) 1770 * rotOff / 180);
//            }
//            drive.driveByEncoder(0, 0, 0, 0);
        }

    }

    /**
     * Smart sleep with opMode running check.
     * @param ms Timeout in milliseconds.
     */
    private void sleepFor(long ms) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.milliseconds() < ms));
    }
}
