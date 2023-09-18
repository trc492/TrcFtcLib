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
     * @return exposure mode, null if unsuccessful.
     */
    public ExposureControl.Mode getExposureMode()
    {
        ExposureControl.Mode exposureMode = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            exposureMode = visionPortal.getCameraControl(ExposureControl.class).getMode();
        }

        return exposureMode;
    }   //getExposureMode

    /**
     * This method sets the exposure mode.
     *
     * @param exposureMode specifies the exposure mode.
     * @return true if successful, false otherwise.
     */
    public boolean setExposureMode(ExposureControl.Mode exposureMode)
    {
        boolean success = false;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            success = visionPortal.getCameraControl(ExposureControl.class).setMode(exposureMode);
        }

        return success;
    }   //setExposureMode

    /**
     * This method returns the camera min and max exposure setting.
     *
     * @return array containing min and max exposure times in msec, null if unsuccessful.
     */
    public int[] getExposureSetting()
    {
        int[] exposureTimes = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            exposureTimes = new int[2];
            exposureTimes[0] = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
            exposureTimes[1] = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        }

        return exposureTimes;
    }   //getExposureSetting

    /**
     * This method returns the current camera exposure time in msec.
     *
     * @return current exposure time in msec, 0 if unsuccessful.
     */
    public int getCurrentExposure()
    {
        int currExposure = 0;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            currExposure =
                (int) visionPortal.getCameraControl(ExposureControl.class).getExposure(TimeUnit.MILLISECONDS);
        }

        return currExposure;
    }   //getCurrentExposure

    /**
     * This method returns the camera min and max gain setting.
     *
     * @return array containing min and max gain values, null if unsuccessful.
     */
    public int[] getGainSetting()
    {
        int[] gains = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            gains = new int[2];
            gains[0] = (int) gainControl.getMinGain();
            gains[1] = (int) gainControl.getMaxGain();
        }

        return gains;
    }   //getGainSetting

    /**
     * This method returns the current camera gain.
     *
     * @return current gain, 0 if unsuccessful.
     */
    public int getCurrentGain()
    {
        int currGain = 0;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            currGain = (int) visionPortal.getCameraControl(GainControl.class).getGain();
        }

        return currGain;
    }   //getCurrentGain

    /**
     * This method sets the camera exposure and gain. This can only be done if the camera is in manual exposure mode.
     * If the camera is not already in manual exposure mode, this method will switch the camera to manual exposure mode.
     * Note: This is a synchronous call and may take some time, so it should only be called at robotInit time.
     *
     * @param exposureMS specifies the exposure time in msec.
     * @param gain specifies the camera gain.
     * @return true if successful, false otherwise.
     */
    public boolean setManualExposure(int exposureMS, int gain)
    {
        boolean success = false;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual)
            {
                success = exposureControl.setMode(ExposureControl.Mode.Manual);
                TrcTimer.sleep(50);
            }

            if (success)
            {
                success = exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                TrcTimer.sleep(20);
            }

            if (success)
            {
                success = visionPortal.getCameraControl(GainControl.class).setGain(gain);
                TrcTimer.sleep(20);
            }
        }

        return success;
    }   //setManualExposure

}   //class FtcVision
