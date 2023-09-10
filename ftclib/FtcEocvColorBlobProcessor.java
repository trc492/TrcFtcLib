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

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;

/**
 * This class implements a vision processor on top of an EOCV color blob pipeline.
 */

public class FtcEocvColorBlobProcessor implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>,
                                                  VisionProcessor
{
    private final TrcOpenCvColorBlobPipeline colorBlobPipeline;
    private final Paint linePaint;
    private boolean annotate = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param colorConversion specifies color space conversion (Imgproc.COLOR_*).
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcEocvColorBlobProcessor(
        String instanceName, int colorConversion, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, TrcDbgTrace tracer)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(
            instanceName, colorConversion, colorThresholds, filterContourParams, tracer);
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeWidth(3);
    }   //FtcEocvColorBlobProcessor

    /**
     * This method returns the pipeline instance name.
     *
     * @return pipeline instance Name
     */
    @Override
    public String toString()
    {
        return colorBlobPipeline.toString();
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
        colorBlobPipeline.reset();
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public TrcOpenCvColorBlobPipeline.DetectedObject[] process(Mat input)
    {
        return colorBlobPipeline.process(input);
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public TrcOpenCvColorBlobPipeline.DetectedObject[] getDetectedObjects()
    {
        return colorBlobPipeline.getDetectedObjects();
    }   //getDetectedObjects

    /**
     * This method enables/disables image annotation of the detected object.
     *
     * @param enabled specifies true to enable annotation, false to disable.
     */
    @Override
    public void setAnnotateEnabled(boolean enabled)
    {
        annotate = enabled;
    }   //setAnnotateEnabled

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        return annotate;
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     * Note: FTC supports multiple vision processors, so we don't control video output. Let's throw an exception here.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     */
    @Override
    public void setVideoOutput(int intermediateStep)
    {
        throw new UnsupportedOperationException("FTC does not support setting video output.");
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     * Note: FTC supports multiple vision processors, so we don't control video output. Let's throw an exception here.
     */
    @Override
    public void setNextVideoOutput()
    {
        throw new UnsupportedOperationException("FTC does not support setting video output.");
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
        return colorBlobPipeline.getIntermediateOutput(step);
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return colorBlobPipeline.getSelectedOutput();
    }   //getSelectedOutput

    //
    // Implements VisionProcessor interface.
    //

   /**
    * This method is called to initialize the vision processor.
    *
    * @param width specifies the image width.
    * @param height specifies the image height.
    * @param calibration specifies the camera calibration data.
    */
    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // Don't really need to do anything here.
    }   //init

   /**
    * This method is called to process an image frame.
    *
    * @param frame specifies the source image to be processed.
    * @param captureTimeNanos specifies the capture frame timestamp.
    * @return array of detected objects.
    */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        return colorBlobPipeline.process(frame);
    }   //processFrame

    @Override
    public synchronized void onDrawFrame(
        Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
        Object userContext)
    {
        // Only one draw operation at a time thank you very much.
        // (we could be called from two different threads - viewport or camera stream)
        if (annotate && userContext != null)
        {
            TrcOpenCvColorBlobPipeline.DetectedObject[] dets =
                (TrcOpenCvColorBlobPipeline.DetectedObject[]) userContext;

            for (TrcOpenCvColorBlobPipeline.DetectedObject object : dets)
            {
                org.opencv.core.Rect objRect = object.getRect();
                float left = objRect.x, right = objRect.x + objRect.width;
                float top = objRect.y, bottom = objRect.y + objRect.height;
                canvas.drawLine(left, top, right, top, linePaint);
                canvas.drawLine(right, top, right, bottom, linePaint);
                canvas.drawLine(right, bottom, left, bottom, linePaint);
                canvas.drawLine(left, bottom, left, top, linePaint);
            }
        }
    }   //onDrawFrame

}  //class FtcEocvColorBlobProcessor
