/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcFtcLib.ftclib;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements an AprilTag detector using EasyOpenCV.
 */
public class FtcAprilTagDetector extends OpenCvPipeline
{
    protected static final String moduleName = "FtcAprilTagDetector";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide a method to return the detected object rect.
     */
    public static class DetectedObject extends TrcVisionTargetInfo.ObjectInfo
    {
        public AprilTagDetection aprilTagInfo;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param aprilTagInfo specifies the detected april tag info.
         */
        public DetectedObject(AprilTagDetection aprilTagInfo)
        {
            this.aprilTagInfo = aprilTagInfo;
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            double width = (Math.abs(aprilTagInfo.corners[0].x - aprilTagInfo.corners[1].x) +
                            Math.abs(aprilTagInfo.corners[2].x - aprilTagInfo.corners[3].x))/2.0;
            double height = (Math.abs(aprilTagInfo.corners[1].y - aprilTagInfo.corners[2].y) +
                             Math.abs(aprilTagInfo.corners[0].y - aprilTagInfo.corners[3].y))/2.0;
            return new Rect((int) (aprilTagInfo.center.x - width/2.0), (int) (aprilTagInfo.center.y - height/2.0),
                            (int) width, (int) height);
        }   //getRect

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "{id=%d,hamming=%d,decisionMargin=%.1f,center=%.1f/%.1f,rect=%s}",
                aprilTagInfo.id, aprilTagInfo.hamming, aprilTagInfo.decisionMargin,
                aprilTagInfo.center.x, aprilTagInfo.center.y, getRect());
        }   //toString

    }   //class DetectedObject

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(AprilTagDetection object);
    }   //interface FilterTarget

    /*
     * A simple container to hold both rotation and translation vectors, which together form a 6DOF pose.
     */
    static class SixDofPose
    {
        Mat rvec;
        Mat tvec;

        public SixDofPose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }   //SixDofPose

        public SixDofPose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }   //class SixDofPose

    private final String instanceName;
    private final int imageWidth, imageHeight;
    private final OpenCvCamera openCvCamera;
    private final boolean showAprilTagView;
    private final TrcDbgTrace tracer;
    private final TrcHomographyMapper homographyMapper;
    private boolean aprilTagEnabled = false;
    private double totalTime = 0.0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    private static final float DEF_DECIMATION = 3.0f;
    private static final int NUM_THREADS = 3;
    private static final Scalar blue = new Scalar(7, 197, 235, 255);
    private static final Scalar red = new Scalar(255,0,0,255);
    private static final Scalar green = new Scalar(0,255,0,255);
    private static final Scalar white = new Scalar(255,255,255,255);

    // UNITS ARE METERS
    private final double tagSize;
    private final double tagSizeX;
    private final double tagSizeY;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;

    private Mat cameraMatrix;
    private long nativeApriltagPtr;
    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detectionsUpdate = null;
    private final Object detectionsUpdateSync = new Object();
    private final Object decimationSync = new Object();
    private float decimation;
    private boolean needToSetDecimation;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the camera image width.
     * @param imageHeight specifies the camera image height.
     * @param cameraRect specifies the homography camera pixel rectangle, can be null if not provided.
     * @param worldRect specifies the homography world coordinate rectangle, can be null if not provided.
     * @param openCvCam specifies the OpenCV camera object.
     * @param cameraRotation specifies the camera orientation.
     * @param showAprilTagView specifies true to show the annotated image on robot controller screen, false to hide it.
     * @param tracer specifies the tracer for trace info, null if none provided.
     * @param tagFamily specifies the tag family.
     * @param tagSize size of the tag in meters.
     * @param fx lens focal length x from camera calibration.
     * @param fy lens focal length y from camera calibration.
     * @param cx lens principal point x from camera calibration.
     * @param cy lens principal point y from camera calibration.
     */
    public FtcAprilTagDetector(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCam, OpenCvCameraRotation cameraRotation, boolean showAprilTagView, TrcDbgTrace tracer,
        AprilTagDetectorJNI.TagFamily tagFamily, double tagSize, double fx, double fy, double cx, double cy)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.openCvCamera = openCvCam;
        this.showAprilTagView = showAprilTagView;
        this.tracer = tracer;

        if (cameraRect != null && worldRect != null)
        {
           homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
           homographyMapper = null;
        }

        this.tagSize = tagSize;
        this.tagSizeX = tagSize;
        this.tagSizeY = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(tagFamily.string, DEF_DECIMATION, NUM_THREADS);

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                openCvCamera.startStreaming(imageWidth, imageHeight, cameraRotation);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        openCvCamera.pauseViewport();
    }   //FtcAprilTagDetector

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method pauses/resumes pipeline processing.
     *
     * @param enabled specifies true to start pipeline processing, false to stop.
     */
    public void setEnabled(boolean enabled)
    {
        if (enabled && !aprilTagEnabled)
        {
            detectionsUpdate = null;
            totalTime = 0.0;
            totalFrames = 0;
            taskStartTime = TrcUtil.getCurrentTime();

            openCvCamera.setPipeline(this);
            if (showAprilTagView)
            {
                openCvCamera.resumeViewport();
            }
        }
        else if (!enabled && aprilTagEnabled)
        {
            openCvCamera.pauseViewport();
            openCvCamera.setPipeline(null);
            detectionsUpdate = null;
        }

        aprilTagEnabled = enabled;
    }   //setEnabled

    /**
     * This method returns the state of AprilTagVision.
     *
     * @return true if the AprilTagVision is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return aprilTagEnabled;
    }   //isTaskEnabled

    /**
     * This method returns an array of detected targets from EasyOpenCV vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return array of detected target info.
     */
    @SuppressWarnings("unchecked")
    public TrcVisionTargetInfo<DetectedObject>[] getDetectedTargetsInfo(
        FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        final String funcName = "getDetectedTargetsInfo";
        TrcVisionTargetInfo<DetectedObject>[] targets = null;
        ArrayList<AprilTagDetection> detectedObjs = getDetectionsUpdate();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API,
                "filter=%s,comparator=%s,objHeightOffset=%.1f,cameraHeight=%.1f",
                filter != null, comparator != null, objHeightOffset, cameraHeight);
        }

        if (detectedObjs != null && detectedObjs.size() > 0)
        {
            ArrayList<TrcVisionTargetInfo<DetectedObject>> targetList = new ArrayList<>();

            for (AprilTagDetection object : detectedObjs)
            {
                if (filter == null || filter.validateTarget(object))
                {
                    TrcVisionTargetInfo<DetectedObject> targetInfo = new TrcVisionTargetInfo<>(
                        new DetectedObject(object), imageWidth, imageHeight, homographyMapper,
                        objHeightOffset, cameraHeight);
                    targetList.add(targetInfo);
                }
            }

            if (targetList.size() > 0)
            {
                targets = targetList.toArray(new TrcVisionTargetInfo[0]);
                if (comparator != null && targets.length > 1)
                {
                    Arrays.sort(targets, comparator);
                }
            }

            if (targets != null && tracer != null)
            {
                for (int i = 0; i < targets.length; i++)
                {
                    tracer.traceInfo(funcName, "[%d] Target=%s", i, targets[i]);
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", targets == null? 0: targets.length);
        }

        return targets;
    }   //getDetectedTargetsInfo

    //
    // Implements OpenCvPipeline abstract method.
    //

    /**
     * This method is called by the garbage collector before the deletion of this object so that we can clean up.
     * This is useful to release resources that Java garbage collector does not deal with.
     */
    @Override
    protected void finalize()
    {
        final String funcName = "finalize";

        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            TrcDbgTrace.getGlobalTracer().traceWarn(funcName, "nativeApriltagPtr was NULL.");
        }
    }   //finalize

    /**
     * This method is called by OpenCvPipeline to process a video frame.
     *
     * @param input specifies the video frame to be processed.
     *
     * @return the video frame to be displayed.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        final String funcName = "processFrame";

        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if (needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        double startTime = TrcUtil.getCurrentTime();
        ArrayList<AprilTagDetection> detections =
            AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagSize, fx, fy, cx, cy);
        double elapsedTime = TrcUtil.getCurrentTime() - startTime;
        totalTime += elapsedTime;
        totalFrames++;
        if (tracer != null)
        {
            tracer.traceInfo(
                funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f",
                totalTime/totalFrames, totalFrames/(TrcUtil.getCurrentTime() - taskStartTime));
        }

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (AprilTagDetection detection : detections)
        {
            SixDofPose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSizeX, tagSizeY);
            drawAxisMarker(input, tagSizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSizeX, tagSizeX, tagSizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }   //processFrame

    /**
     * This method sets the decimation parameter of the AprilTag detector.
     *
     * @param decimation specifies the new decimation value.
     */
    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }   //setDecimation

    /**
     * This method returns the detected object array in a thread safe manner.
     *
     * @return an array list of the detected object.
     */
    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }   //getDetectionUpdate

    /**
     * This method constructs the camera matrix.
     */
    private void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }   //constructMatrix

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param thickness the thickness of the lines drawn.
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
            new Point3(0, 0, 0),
            new Point3(length,0,0),
            new Point3(0,length,0),
            new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }   //drawAxisMarker

    /**
     * Draw a 3D cube marker on a detection.
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the cube depth.
     * @param tagWidth the cube width.
     * @param tagHeight the cube height.
     * @param thickness the thickness of the lines drawn.
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private void draw3dCubeMarker(
        Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
            new Point3(-tagWidth/2, tagHeight/2,0),
            new Point3( tagWidth/2, tagHeight/2,0),
            new Point3( tagWidth/2,-tagHeight/2,0),
            new Point3(-tagWidth/2,-tagHeight/2,0),
            new Point3(-tagWidth/2, tagHeight/2,-length),
            new Point3( tagWidth/2, tagHeight/2,-length),
            new Point3( tagWidth/2,-tagHeight/2,-length),
            new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for (int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }   //draw3dCubeMarker

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    private SixDofPose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        SixDofPose pose = new SixDofPose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }   //poseFromTrapezoid

}  //class FtcAprilTagDetector
