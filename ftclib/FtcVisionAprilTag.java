/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class encapsulates the AprilTag vision processor to make all vision processors conform to our framework
 * library. By doing so, one can switch between different vision processors and have access to a common interface.
 */
public class FtcVisionAprilTag
{
    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide methods to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public AprilTagDetection aprilTagDetection;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param aprilTagDetection specifies the detected april tag object.
         */
        public DetectedObject(AprilTagDetection aprilTagDetection)
        {
            this.aprilTagDetection = aprilTagDetection;
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
        public Rect getObjectRect()
        {
            // Calculate rect from AprilTag detection corner points.
            return getDetectedRect(aprilTagDetection);
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            // AprilTag detection does not provide area, just calculate it from rect.
            return getDetectedRect(aprilTagDetection).area();
        }   //getObjectArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose2D getObjectPose()
        {
            TrcPose2D pose = null;

            if (aprilTagDetection.ftcPose != null)
            {
                // Get pose from AprilTag detection ftcPose.
                pose = new TrcPose2D(
                    aprilTagDetection.ftcPose.x, aprilTagDetection.ftcPose.y, aprilTagDetection.ftcPose.yaw);
            }

            return pose;
        }   //getObjectPose

        /**
         * This method returns the real world width of the detected object.
         *
         * @return real world width of the detected object.
         */
        @Override
        public Double getObjectWidth()
        {
            // AprilTag detection does not provide detected object width.
            return null;
        }   //getObjectWidth

        /**
         * This method returns the real world depth of the detected object.
         *
         * @return real world depth of the detected object.
         */
        @Override
        public Double getObjectDepth()
        {
            // AprilTag detection does not provide detected object depth.
            return null;
        }   //getObjectDepth

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            if (aprilTagDetection.ftcPose != null)
            {
                return String.format(
                    Locale.US,
                    "{id=%d,center=%.1f/%.1f,rect=%s,ftcPose=(x=%.1f,y=%.1f,z=%.1f,yaw=%.1f,pitch=%.1f,roll=%.1f," +
                    "range=%.1f,bearing=%.1f,elevator=%.1f),fieldPos=%s,hamming=%d,decisionMargin=%.1f}",
                    aprilTagDetection.id, aprilTagDetection.center.x, aprilTagDetection.center.y, getObjectRect(),
                    aprilTagDetection.ftcPose.x, aprilTagDetection.ftcPose.y, aprilTagDetection.ftcPose.z,
                    aprilTagDetection.ftcPose.yaw, aprilTagDetection.ftcPose.pitch, aprilTagDetection.ftcPose.roll,
                    aprilTagDetection.ftcPose.range, aprilTagDetection.ftcPose.bearing,
                    aprilTagDetection.ftcPose.elevation, aprilTagDetection.metadata.fieldPosition,
                    aprilTagDetection.hamming, aprilTagDetection.decisionMargin);
            }
            else
            {
                return String.format(
                    Locale.US,
                    "{id=%d,center=%.1f/%.1f,rect=%s,fieldPos=%s,hamming=%d,decisionMargin=%.1f}",
                    aprilTagDetection.id, aprilTagDetection.center.x, aprilTagDetection.center.y, getObjectRect(),
                    aprilTagDetection.hamming, aprilTagDetection.decisionMargin);
            }
        }   //toString

    }   //class DetectedObject

    /**
     * This class encapsulates all the parameters for creating the AprilTag vision processor. If this is not used,
     * all default parameters will be applied.
     */
    public static class Parameters
    {
        boolean drawTagId = true;
        boolean drawTagOutline = true;
        boolean drawAxes = false;
        boolean drawCubeProjection = false;
        double[] lensIntrinsics = null;
        DistanceUnit distanceUnit = DistanceUnit.INCH;
        AngleUnit angleUnit = AngleUnit.DEGREES;

        public Parameters setDrawTagIdEnabled(boolean enabled)
        {
            this.drawTagId = enabled;
            return this;
        }   //setDrawTagIdEnabled

        public Parameters setDrawTagOutlineEnabled(boolean enabled)
        {
            this.drawTagOutline = enabled;
            return this;
        }   //setDrawTagOutlineEnabled

        public Parameters setDrawAxesEnabled(boolean enabled)
        {
            this.drawAxes = enabled;
            return this;
        }   //setDrawAxesEnabled

        public Parameters setDrawCubeProjectionEnabled(boolean enabled)
        {
            this.drawCubeProjection = enabled;
            return this;
        }   //setDrawCubeProjectionEnabled

        public Parameters setLensIntrinsics(double fx, double fy, double cx, double cy)
        {
            this.lensIntrinsics = new double[] {fx, fy, cx, cy};
            return this;
        }   //setLensIntrinsics

        public Parameters setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit)
        {
            this.distanceUnit = distanceUnit;
            this.angleUnit = angleUnit;
            return this;
        }   //setOutputUnits

    }   //class Parameters

    private final String instanceName;
    private final TrcDbgTrace tracer;
    private final AprilTagProcessor aprilTagProcessor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the AprilTag parameters, can be null if using default parameters.
     * @param tagFamily specifies the tag family.
     */
    public FtcVisionAprilTag(Parameters params, AprilTagProcessor.TagFamily tagFamily)
    {
        instanceName = tagFamily.name();
        tracer = new TrcDbgTrace();
        // Create the AprilTag processor.
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder().setTagFamily(tagFamily);
        if (params != null)
        {
            if (params.lensIntrinsics != null)
            {
                builder.setLensIntrinsics(
                    params.lensIntrinsics[0], params.lensIntrinsics[1],
                    params.lensIntrinsics[2], params.lensIntrinsics[3]);
            }
            builder
                .setDrawTagID(params.drawTagId)
                .setDrawTagOutline(params.drawTagOutline)
                .setDrawAxes(params.drawAxes)
                .setDrawCubeProjection(params.drawCubeProjection)
                .setOutputUnits(params.distanceUnit, params.angleUnit);
        }
        aprilTagProcessor = builder.build();
    }   //FtcVisionAprilTag

    /**
     * This method returns the tag family string.
     *
     * @return tag family string.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns its tracer used for tracing info.
     *
     * @return tracer.
     */
    public TrcDbgTrace getTracer()
    {
        return tracer;
    }   //getTracer

    /**
     * This method returns the AprilTag vision processor.
     *
     * @return AprilTag vision processor.
     */
    public AprilTagProcessor getVisionProcessor()
    {
        return aprilTagProcessor;
    }   //getVisionProcessor

    /**
     * This method returns the target info of the given detected target.
     *
     * @param detection specifies the detected target.
     * @return information about the detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getDetectedTargetInfo(AprilTagDetection detection)
    {
        TrcVisionTargetInfo<DetectedObject> targetInfo = new TrcVisionTargetInfo<>(new DetectedObject(detection));
        tracer.traceDebug(instanceName, "TargetInfo=" + targetInfo);

        return targetInfo;
    }   //getDetectedTargetInfo

    /**
     * This method returns an array of target info on the filtered detected targets.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return sorted target info array.
     */
    public TrcVisionTargetInfo<DetectedObject>[] getDetectedTargetsInfo(
        Integer id, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator)
    {
        TrcVisionTargetInfo<DetectedObject>[] targetsInfo = null;
        ArrayList<AprilTagDetection> targets = aprilTagProcessor.getFreshDetections();

        if (targets != null)
        {
            ArrayList<TrcVisionTargetInfo<DetectedObject>> targetsList = new ArrayList<>();

            for (AprilTagDetection detection : targets)
            {
                // Check for ID match if provided.
                if (id == null || id == detection.id)
                {
                    targetsList.add(getDetectedTargetInfo(detection));
                }
            }

            if (!targetsList.isEmpty())
            {
                targetsInfo = new TrcVisionTargetInfo[targetsList.size()];
                targetsList.toArray(targetsInfo);

                if (comparator != null)
                {
                    Arrays.sort(targetsInfo, comparator);
                }
            }
        }

        return targetsInfo;
    }   //getDetectedTargetsInfo

    /**
     * This method returns the target info of the best detected target.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getBestDetectedTargetInfo(
        Integer id, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator)
    {
        TrcVisionTargetInfo<DetectedObject> bestTarget = null;
        TrcVisionTargetInfo<DetectedObject>[] detectedTargets = getDetectedTargetsInfo(id, comparator);

        if (detectedTargets != null && detectedTargets.length > 0)
        {
            bestTarget = detectedTargets[0];
        }

        return bestTarget;
    }   //getBestDetectedTargetInfo

}   //class FtcVisionAprilTag
