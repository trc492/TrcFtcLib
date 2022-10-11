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

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvPipeline;

/**
 * This class implements an EOCV color blob pipeline.
 */
public class FtcEocvColorBlobPipeline extends OpenCvPipeline
                                      implements TrcOpenCvPipeline<TrcOpenCvColorBlobPipeline.DetectedObject>
{
    private final TrcOpenCvColorBlobPipeline colorBlobPipeline;
    private boolean showColorFilterOutput = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param useHsv specifies true for HSV color space, false for RGB.
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FtcEocvColorBlobPipeline(
        String instanceName, boolean useHsv, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, TrcDbgTrace tracer)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(
            instanceName, useHsv, colorThresholds, filterContourParams, tracer);
    }   //FtcEocvColorBlobPipeline

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
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     */
    @Override
    public void process(Mat input)
    {
        colorBlobPipeline.process(input);
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
     * This method toggles between annotated input or color filter output on the display.
     *
     * @return the updated state.
     */
    public boolean toggleColorFilterOutput()
    {
        showColorFilterOutput = !showColorFilterOutput;
        return showColorFilterOutput;
    }   //toggleColorFilterOutput

    //
    // Implements OpenCvPipeline abstract methods.
    //

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
        return showColorFilterOutput? colorBlobPipeline.getColorThresholdOutput(): input;
    }   //processFrame

}  //class FtcEocvColorBlobPipeline
