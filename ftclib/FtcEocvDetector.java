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

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements an EasyOpenCV detector. Typically, it is extended by a specific detector that provides the
 * pipeline to process an image for detecting objects using OpenCV APIs.
 */
public class FtcEocvDetector
{
    private final String instanceName;
    private final OpenCvCamera openCvCamera;
    private final int imageWidth, imageHeight;
    private final boolean showEocvView;
    private final TrcDbgTrace tracer;
    private final TrcHomographyMapper homographyMapper;

    private boolean eocvEnabled = false;
    private volatile TrcOpenCvPipeline<?> openCvPipeline = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param openCvCamera specifies the camera object.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRotation specifies the camera orientation.
     * @param showEocvView specifies true to show the annotated image on robot controller screen, false to hide it.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcEocvDetector(
        String instanceName, OpenCvCamera openCvCamera, int imageWidth, int imageHeight,
        OpenCvCameraRotation cameraRotation, boolean showEocvView, TrcHomographyMapper.Rectangle cameraRect,
        TrcHomographyMapper.Rectangle worldRect, TrcDbgTrace tracer)
    {
        this.instanceName = instanceName;
        this.openCvCamera = openCvCamera;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.showEocvView = showEocvView;
        this.tracer = tracer;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }

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

        openCvCamera.showFpsMeterOnViewport(false);
        openCvCamera.pauseViewport();
    }   //FtcEocvDetector

    /**
     * This method sets the EOCV pipeline to be used for the detection.
     *
     * @param pipeline specifies the pipeline to be used for detection.
     */
    public void setPipeline(TrcOpenCvPipeline<?> pipeline)
    {
        openCvPipeline = pipeline;
    }   //setPipeline

    /**
     * This method returns the current active pipeline.
     *
     * @return current active pipeline, null if no active pipeline.
     */
    public TrcOpenCvPipeline<?> getPipeline()
    {
        return openCvPipeline;
    }   //getPipeline

    /**
     * This method pauses/resumes pipeline processing.
     *
     * @param enabled specifies true to start pipeline processing, false to stop.
     */
    public void setEnabled(boolean enabled)
    {
        if (enabled && !eocvEnabled)
        {
            if (openCvPipeline != null)
            {
                openCvPipeline.performanceMetrics.reset();
            }

            openCvCamera.setPipeline((OpenCvPipeline) openCvPipeline);
            if (showEocvView)
            {
                openCvCamera.resumeViewport();
            }
        }
        else if (!enabled && eocvEnabled)
        {
            if (showEocvView)
            {
                openCvCamera.pauseViewport();
            }
            openCvCamera.setPipeline(null);
        }

        eocvEnabled = enabled;
    }   //setEnabled

    /**
     * This method returns the state of EocvVision.
     *
     * @return true if the EocvVision is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return eocvEnabled;
    }   //isEnabled

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
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] getDetectedTargetsInfo(
        TrcOpenCvDetector.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        final String funcName = instanceName + ".getDetectedTargetsInfo";
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] detectedTargets = null;

        TrcOpenCvDetector.DetectedObject<?>[] objects =
            (TrcOpenCvDetector.DetectedObject<?>[]) openCvPipeline.getDetectedObjects();

        if (objects != null)
        {
            ArrayList<TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>> targetList = new ArrayList<>();

            for (TrcOpenCvDetector.DetectedObject<?> obj : objects)
            {
                if (filter == null || filter.validateTarget(obj))
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> targetInfo =
                        new TrcVisionTargetInfo<>(
                            obj, imageWidth, imageHeight, homographyMapper, objHeightOffset, cameraHeight);
                    targetList.add(targetInfo);
                }
            }

            if (targetList.size() > 0)
            {
                detectedTargets = targetList.toArray(new TrcVisionTargetInfo[0]);
                if (comparator != null && detectedTargets.length > 1)
                {
                    Arrays.sort(detectedTargets, comparator);
                }
            }

            if (detectedTargets != null && tracer != null)
            {
                for (int i = 0; i < detectedTargets.length; i++)
                {
                    tracer.traceInfo(funcName, "[%d] Target=%s", i, detectedTargets[i]);
                }
            }
        }

        return detectedTargets;
    }   //getDetectedTargetsInfo

}   //class FtcEocvDetector
