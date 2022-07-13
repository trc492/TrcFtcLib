/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements an EasyOpenCV detector. Typically, it is extended by a specific detector that provides
 * the algorithm to process an image for detecting objects using OpenCV APIs.
 */
public abstract class FtcEocvDetector extends OpenCvPipeline
{
    protected static final String moduleName = "FtcEocvDetector";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method is provided by the subclass to return an array of detected objects.
     *
     * @return array of detected objects.
     */
    public abstract TrcOpenCVDetector.DetectedObject[] getDetectedObjects();

    private final String instanceName;
    private final int imageWidth, imageHeight;
    private final TrcDbgTrace tracer;
    private final TrcHomographyMapper homographyMapper;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcEocvDetector(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect, TrcDbgTrace tracer)
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
        this.tracer = tracer;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
    }   //FtcEocvDetector

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
     * This method returns an array of detected targets from EasyOpenCV vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return array of detected target info.
     */
    @SuppressWarnings("unchecked")
    public TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject>[] getDetectedTargetsInfo(
        TrcOpenCVDetector.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject>> comparator)
    {
        final String funcName = "getDetectedTargetsInfo";
        TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject>[] targets = null;
        TrcOpenCVDetector.DetectedObject[] detectedObjs = getDetectedObjects();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "filter=%s,comparator=%s", filter != null, comparator != null);
        }

        if (detectedObjs != null)
        {
            ArrayList<TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject>> targetList = new ArrayList<>();

            for (TrcOpenCVDetector.DetectedObject detectedObj : detectedObjs)
            {
                if (filter == null || filter.validateTarget(detectedObj))
                {
                    TrcVisionTargetInfo<TrcOpenCVDetector.DetectedObject> targetInfo =
                        new TrcVisionTargetInfo<>(detectedObj, imageWidth, imageHeight, homographyMapper);
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

}   //class FtcEocvDetector
