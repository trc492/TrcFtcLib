/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class makes using TensorFlow a little easier by minimizing the number of calls to it. It only exposes the
 * minimum things you need to set for the FTC competition. If you want to do more complex stuff, you may consider
 * not using this and call TensorFlow directly so you can customize other stuff. This class provides methods to
 * simplify getting detected object info from TensorFlow.
 */
public class FtcTensorFlow
{
    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide a method to return the detected object rect.
     */
    public static class DetectedObject extends TrcVisionTargetInfo.ObjectInfo
    {
        public final String label;
        public final Rect rect;
        public final double angle;
        public final double confidence;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the object label.
         * @param rect specifies the rect of the object.
         * @param angle specifies an estimation of the horizontal angle to the detected object
         * @param confidence specifies the confidence of the detection.
         */
        public DetectedObject(String label, Rect rect, double angle, double confidence)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            return rect;
        }   //getRect

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "{" + label + "," + rect.toString() + ",angle=" + angle + ",confidence=" + confidence + "}";
        }   //toString

    }   //class DetectedObject

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(Recognition object);
    }   //interface FilterTarget

    private final TrcDbgTrace tracer;
    private final TFObjectDetector tfod;
    private final TrcHomographyMapper homographyMapper;
    private boolean enabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param vuforia specifies the FtcVuforia object.
     * @param tfodParams specifies the TensorFlow parameters.
     * @param modelAsset specifies the model asset file name.
     * @param objectLabels specifies the names of detectable objects.
     * @param cameraRect specifies the camera rectangle for Homography Mapper.
     * @param worldRect specifies the world rectangle for Homography Mapper.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcTensorFlow(
        FtcVuforia vuforia, TFObjectDetector.Parameters tfodParams, String modelAsset, String[] objectLabels,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect, TrcDbgTrace tracer)
    {
        this.tracer = tracer;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParams, vuforia.getLocalizer());
        tfod.loadModelFromAsset(modelAsset, objectLabels);

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
    }   //FtcTensorFlow

    /**
     * Constructor: Create an instance of the object.
     *
     * @param vuforia specifies the FtcVuforia object.
     * @param tfodParams specifies the TensorFlow parameters.
     * @param modelAsset specifies the model asset file name.
     * @param objectLabels specifies the names of detectable objects.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcTensorFlow(
        FtcVuforia vuforia, TFObjectDetector.Parameters tfodParams, String modelAsset, String[] objectLabels,
        TrcDbgTrace tracer)
    {
        this(vuforia, tfodParams, modelAsset, objectLabels, null, null, tracer);
    }   //FtcTensorFlow

    /**
     * Constructor: Create an instance of the object.
     *
     * @param vuforia specifies the FtcVuforia object.
     * @param tfodParams specifies the TensorFlow parameters.
     * @param modelAsset specifies the model asset file name.
     * @param objectLabels specifies the names of detectable objects.
     */
    public FtcTensorFlow(
        FtcVuforia vuforia, TFObjectDetector.Parameters tfodParams, String modelAsset, String[] objectLabels)
    {
        this(vuforia, tfodParams, modelAsset, objectLabels, null, null, null);
    }   //FtcTensorFlow

    /**
     * This method enables/disables TensorFlow.
     *
     * @param enabled specifies true to enable TensorFlow, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        if (!this.enabled && enabled)
        {
            tfod.activate();
        }
        else if (this.enabled && !enabled)
        {
            tfod.deactivate();
        }
        this.enabled = enabled;
    }   //setEnabled

    /**
     * This method checks if TensorFlow is enabled.
     *
     * @return true if TensorFlow is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method shuts down TensorFlow.
     */
    public void shutdown()
    {
        setEnabled(false);
        tfod.shutdown();
    }   //shutdown

    /**
     * This method sets the zoom parameters.
     *
     * @param magnification specifies the magnification factor, 1.0 for full camera view.
     * @param aspectRatio specifies the aspect ratio of the zoom region.
     */
    public void setZoom(double magnification, double aspectRatio)
    {
        tfod.setZoom(magnification, aspectRatio);
    }   //setZoom

    /**
     * This method returns an array list of detected targets. If a target label is given, only detected targets with
     * the same label will be returned.
     *
     * @param label specifies the target label to filter the target list, can be null if no filtering.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @return filtered target array list.
     */
    private ArrayList<Recognition> getDetectedTargets(String label, FilterTarget filter)
    {
        final String funcName = "getDetectedTargets";
        ArrayList<Recognition> targets = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            targets = new ArrayList<>();
            for (int i = 0; i < updatedRecognitions.size(); i++)
            {
                Recognition object = updatedRecognitions.get(i);
                boolean foundIt = label == null || label.equals(object.getLabel());
                boolean rejected = false;

                if (foundIt)
                {
                    if (filter == null || filter.validateTarget(object))
                    {
                        targets.add(object);
                    }
                    else
                    {
                        rejected = true;
                    }
                }

                if (tracer != null)
                {
                    tracer.traceInfo(
                        funcName, "[%d] TensorFlow.%s:x=%.0f,y=%.0f,w=%.0f,h=%.0f,foundIt=%s,rejected=%s",
                        i, object.getLabel(), object.getLeft(), object.getTop(), object.getWidth(),
                        object.getHeight(), foundIt, rejected);
                }
            }

            if (targets.size() == 0)
            {
                //
                // No target found.
                //
                targets = null;
            }
        }

        return targets;
    }   //getDetectedTargets

    /**
     * This method returns the target info of the given detected target.
     *
     * @param target specifies the detected target
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return information about the detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getDetectedTargetInfo(
        Recognition target, double objHeightOffset, double cameraHeight)
    {
        final String funcName = "getTargetInfo";
        TrcVisionTargetInfo<DetectedObject> targetInfo = new TrcVisionTargetInfo<>(
            new DetectedObject(
                target.getLabel(),
                new Rect((int)target.getLeft(), (int)target.getTop(), (int)target.getWidth(), (int)target.getHeight()),
                target.estimateAngleToObject(AngleUnit.DEGREES), target.getConfidence()),
            target.getImageWidth(), target.getImageHeight(), homographyMapper, objHeightOffset, cameraHeight);

        if (tracer != null)
        {
            tracer.traceInfo(funcName, "###TargetInfo###: %s", targetInfo);
        }

        return targetInfo;
    }   //getDetectedTargetInfo

    /**
     * This method returns an array of target info on the filtered detected targets.
     *
     * @param label specifies the target label to filter the target list, can be null if no filtering.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return filtered target info array.
     */
    public TrcVisionTargetInfo<DetectedObject>[] getDetectedTargetsInfo(
        String label, FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        ArrayList<Recognition> targets = getDetectedTargets(label, filter);
        TrcVisionTargetInfo<DetectedObject>[] targetsInfo =
            targets != null && targets.size() > 0? new TrcVisionTargetInfo[targets.size()]: null;

        if (targetsInfo != null)
        {
            for (int i = 0; i < targets.size(); i++)
            {
                targetsInfo[i] = getDetectedTargetInfo(targets.get(i), objHeightOffset, cameraHeight);
            }

            if (comparator != null)
            {
                Arrays.sort(targetsInfo, comparator);
            }
        }

        return targetsInfo;
    }   //getDetectedTargetsInfo

    /**
     * This method maps a camera screen point to the real world point using homography.
     *
     * @param point specifies the camera screen point.
     * @return real world coordinate point.
     */
    public Point mapPoint(Point point)
    {
        return homographyMapper != null? homographyMapper.mapPoint(point): null;
    }   //mapPoint

}   //class FtcTensorFlow
