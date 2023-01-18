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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements an AprilTag pipeline using EasyOpenCV.
 */
public class FtcEocvAprilTagPipeline extends OpenCvPipeline
                                     implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>
{
    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject extends TrcOpenCvDetector.DetectedObject<AprilTagDetection>
    {
        /**
         * Constructor: Creates an instance of the object.
         *
         * @param aprilTagInfo specifies the detected april tag info.
         */
        public DetectedObject(AprilTagDetection aprilTagInfo)
        {
            super(aprilTagInfo);
        }   //DetectedObject

        /**
         * This method calculates the rectangle of the detected AprilTag.
         *
         * @param at specifies the AprilTag info.
         * @return AprilTag rectangle.
         */
        public static Rect getDetectedRect(AprilTagDetection at)
        {
            double width =
                (Math.abs(at.corners[0].x - at.corners[1].x) + Math.abs(at.corners[2].x - at.corners[3].x))/2.0;
            double height =
                (Math.abs(at.corners[1].y - at.corners[2].y) + Math.abs(at.corners[0].y - at.corners[3].y))/2.0;

            return new Rect((int) (
                at.center.x - width/2.0), (int) (at.center.y - height/2.0), (int) width, (int) height);
        }   //getDetectedRect

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            return getDetectedRect(object);
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            return getDetectedRect(object).area();
        }   //getArea

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
                object.id, object.hamming, object.decisionMargin, object.center.x, object.center.y, getRect());
        }   //toString

    }   //class DetectedObject

    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0, 255);
    private static final Scalar ANNOTATE_RECT_WHITE = new Scalar(255, 255, 255, 255);
    private static final int ANNOTATE_RECT_THICKNESS = 3;
    private static final float DEF_DECIMATION = 3.0f;
    private static final int NUM_THREADS = 3;
    private static final Scalar RED = new Scalar(255,0,0,255);
    private static final Scalar GREEN = new Scalar(0,255,0,255);
    private static final Scalar BLUE = new Scalar(7, 197, 235, 255);
    private static final Scalar WHITE = new Scalar(255,255,255,255);

    // UNITS ARE METERS
    private final AprilTagDetectorJNI.TagFamily tagFamily;
    private final double tagSize;
    private final double tagSizeX;
    private final double tagSizeY;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    private final TrcDbgTrace tracer;
    private final Mat cameraMatrix;
    private long nativeAprilTagPtr;
    private final Mat grayMat;
    private final Mat[] intermediateMats;

    private final AtomicReference<DetectedObject[]> detectedObjsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private boolean annotate = false;
    private final Object decimationSync = new Object();
    private float decimation;
    private boolean needToSetDecimation;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param tagFamily specifies the tag family.
     * @param tagSize size of the tag in meters.
     * @param fx lens focal length x from camera calibration.
     * @param fy lens focal length y from camera calibration.
     * @param cx lens principal point x from camera calibration.
     * @param cy lens principal point y from camera calibration.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcEocvAprilTagPipeline(
        AprilTagDetectorJNI.TagFamily tagFamily, double tagSize, double fx, double fy, double cx, double cy,
        TrcDbgTrace tracer)
    {
        this.tagFamily = tagFamily;
        this.tagSize = tagSize;
        this.tagSizeX = tagSize;
        this.tagSizeY = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.tracer = tracer;

        cameraMatrix = constructMatrix();
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(tagFamily.string, DEF_DECIMATION, NUM_THREADS);
        grayMat = new Mat();
        intermediateMats = new Mat[2];
        intermediateMats[0] = null;
        intermediateMats[1] = grayMat;
    }   //FtcEocvAprilTagPipeline

    /**
     * This method returns the tag family string.
     *
     * @return tag family string.
     */
    @Override
    public String toString()
    {
        return tagFamily.string;
    }   //toString

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        performanceMetrics.reset();
        intermediateStep = 0;
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] process(Mat input)
    {
        DetectedObject[] detectedObjects;
        double startTime = TrcTimer.getCurrentTime();

        intermediateMats[0] = input;
        // Convert to grayscale.
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_BGR2GRAY);

        synchronized (decimationSync)
        {
            if (needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        ArrayList<AprilTagDetection> detections =
            AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, grayMat, tagSize, fx, fy, cx, cy);
        performanceMetrics.logProcessingTime(startTime);
        performanceMetrics.printMetrics(tracer);

        detectedObjects = new DetectedObject[detections.size()];
        for (int i = 0; i < detectedObjects.length; i++)
        {
            detectedObjects[i] = new DetectedObject(detections.get(i));
        }

        if (annotate)
        {
            Mat output = getIntermediateOutput(intermediateStep);
            Scalar color = intermediateStep == 0? ANNOTATE_RECT_COLOR: ANNOTATE_RECT_WHITE;
            annotateFrame(output, detectedObjects, color, ANNOTATE_RECT_THICKNESS);
//            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
//            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
//            for (AprilTagDetection detection : detections)
//            {
//                SixDofPose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSizeX, tagSizeY);
//                drawAxisMarker(output, tagSizeY/2.0, 3, pose.rvec, pose.tvec, cameraMatrix);
//                draw3dCubeMarker(output, tagSizeX, tagSizeX, tagSizeY, 3, pose.rvec, pose.tvec, cameraMatrix);
//                Imgproc.rectangle(output, DetectedObject.getDetectedRect(detection), GREEN, 3);
//            }
        }
        detectedObjsUpdate.set(detectedObjects);

        return detectedObjects;
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] getDetectedObjects()
    {
        return detectedObjsUpdate.getAndSet(null);
    }   //getDetectedObjects

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat and optionally annotate the
     * detected rectangle on it.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     */
    @Override
    public void setVideoOutput(int intermediateStep, boolean annotate)
    {
        if (intermediateStep >= 0 && intermediateStep < intermediateMats.length)
        {
            this.intermediateStep = intermediateStep;
            this.annotate = annotate;
        }
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     *
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     */
    @Override
    public void setNextVideoOutput(boolean annotate)
    {
        intermediateStep = (intermediateStep + 1) % intermediateMats.length;
        this.annotate = annotate;
    }   //setNextVideoOutput

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (0 is the original input frame).
     * @return processed frame of the specified step.
     */
    @Override
    public Mat getIntermediateOutput(int step)
    {
        Mat mat = null;

        if (step >= 0 && step < intermediateMats.length)
        {
            mat = intermediateMats[step];
        }

        return mat;
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return getIntermediateOutput(intermediateStep);
    }   //getSelectedOutput

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

        // Might be null if createAprilTagDetector() threw an exception
        if (nativeAprilTagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
            nativeAprilTagPtr = 0;
        }
        else
        {
            TrcDbgTrace.getGlobalTracer().traceWarn(funcName, "nativeAprilTagPtr was NULL.");
        }
    }   //finalize

    /**
     * This method is called by OpenCvPipeline to process an image frame.
     *
     * @param input specifies the image frame to be processed.
     *
     * @return the image frame to be displayed.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        process(input);
        return getSelectedOutput();
    }   //processFrame

    /**
     * This method constructs the camera matrix.
     */
    private Mat constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        Mat camMatrix = new Mat(3, 3, CvType.CV_32FC1);

        camMatrix.put(0, 0, fx);
        camMatrix.put(0, 1, 0);
        camMatrix.put(0, 2, cx);

        camMatrix.put(1, 0, 0);
        camMatrix.put(1, 1, fy);
        camMatrix.put(1, 2, cy);

        camMatrix.put(2, 0, 0);
        camMatrix.put(2, 1, 0);
        camMatrix.put(2, 2, 1);

        return camMatrix;
    }   //constructMatrix

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
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], RED, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], BLUE, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, WHITE, -1);
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
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], BLUE, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], GREEN, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], GREEN, thickness);
    }   //draw3dCubeMarker

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

}  //class FtcEocvAprilTagPipeline
