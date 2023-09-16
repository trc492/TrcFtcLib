/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.TimeUnit;

import TrcCommonLib.trclib.TrcTimer;

/**
 * This class creates an FTC Vision Portal to support multiple vision processors. It also provides methods to
 * configure camera settings.
 */
public class FtcVision
{
    private final VisionPortal visionPortal;
    private final ExposureControl exposureControl;
    private final GainControl gainControl;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param webcamName specifies USB webcam name, null if using phone built-in camera.
     * @param cameraDirection specifies the phone camera direction, null if using USB webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    private FtcVision(
        WebcamName webcamName, BuiltinCameraDirection cameraDirection, int imageWidth, int imageHeight,
        boolean enableLiveView, VisionProcessor... visionProcessors)
    {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (webcamName != null)
        {
            builder.setCamera(webcamName);
        }
        else
        {
            builder.setCamera(cameraDirection);
        }
        builder.setCameraResolution(new Size(imageWidth, imageHeight));

        if (enableLiveView)
        {
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(true);
            //Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //  builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        }
        else
        {
            builder.enableLiveView(false);
        }

        builder.addProcessors(visionProcessors);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
    }   //FtcVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param webcamName specifies USB webcam name, null if using phone built-in camera.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    public FtcVision(
        WebcamName webcamName, int imageWidth, int imageHeight, boolean enableLiveView,
        VisionProcessor... visionProcessors)
    {
        this(webcamName, null, imageWidth, imageHeight, enableLiveView, visionProcessors);
    }   //FtcVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraDirection specifies the phone camera direction, null if using USB webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    public FtcVision(
        BuiltinCameraDirection cameraDirection, int imageWidth, int imageHeight, boolean enableLiveView,
        VisionProcessor... visionProcessors)
    {
        this(null, cameraDirection, imageWidth, imageHeight, enableLiveView, visionProcessors);
    }   //FtcVision

    /**
     * This method returns the created vision portal.
     *
     * @return created vision portal.
     */
    public VisionPortal getVisionPortal()
    {
        return visionPortal;
    }   //getVisionPortal

    /**
     * This method enables/disables the vision processor.
     *
     * @param visionProcessor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable the vision processor, false to disable.
     */
    public void setProcessorEnabled(VisionProcessor visionProcessor, boolean enabled)
    {
        visionPortal.setProcessorEnabled(visionProcessor, enabled);
    }   //setProcessorEnabled

    /**
     * This method checks if the vision processor is enabled.
     *
     * @param visionProcessor specifies the vision processor.
     * @return true if the vision processor is enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor visionProcessor)
    {
        return visionPortal.getProcessorEnabled(visionProcessor);
    }   //isVisionProcessorEnabled

    /**
     * This method returns the camera state.
     *
     * @return camera state.
     */
    public VisionPortal.CameraState getCameraState()
    {
        return visionPortal.getCameraState();
    }   //getCameraState

    /**
     * This method return the camera exposure mode.
     *
     * @return exposure mode.
     */
    public ExposureControl.Mode getExposureMode()
    {
        return exposureControl.getMode();
    }   //getExposureMode

    /**
     * This method sets the exposure mode.
     *
     * @param exposureMode specifies the exposure mode.
     */
    public void setExposureMode(ExposureControl.Mode exposureMode)
    {
        exposureControl.setMode(exposureMode);
    }   //setExposureMode

    /**
     * This method returns the camera min exposure setting.
     *
     * @return min exposure.
     */
    public int getMinExposure()
    {
        return (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
    }   //getMinExposure

    /**
     * This method returns the camera max exposure setting.
     *
     * @return max exposure.
     */
    public int getMaxExposure()
    {
        return (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
    }   //getMaxExposure

    /**
     * This method returns the camera min gain setting.
     *
     * @return min gain.
     */
    public int getMinGain()
    {
        return gainControl.getMinGain();
    }   //getMinGain

    /**
     * This method returns the camera max gain setting.
     *
     * @return max gain.
     */
    public int getMaxGain()
    {
        return gainControl.getMaxGain();
    }   //getMaxGain

    /**
     * This method sets the camera exposure. If the camera is not already in manual exposure mode, this method will
     * switch the camera to manual exposure mode.
     * Note: This is a synchronous call and may take some time, so it should only be called at robotInit time.
     *
     * @param exposureMS specifies the exposure time in msec.
     */
    public void setManualExposure(int exposureMS)
    {
        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
        {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            TrcTimer.sleep(50);
        }

        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        TrcTimer.sleep(20);
    }   //setManualExposure

    /**
     * This method sets the camera gain.
     * Note: This is a synchronous call and may take some time, so it should only be called at robotInit time.
     *
     * @param gain specifies the camera gain.
     */
    public void setGain(int gain)
    {
        gainControl.setGain(gain);
        TrcTimer.sleep(20);
    }   //setGain

}   //class FtcVision
