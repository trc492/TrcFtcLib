/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.graphics.Bitmap;

import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.HashMap;

import TrcCommonLib.trclib.TrcVideoSource;

/**
 * This class makes using Vuforia a little easier by minimizing the number of calls to it. It only exposes the
 * minimum things you need to set for the FTC competition. If you want to do more complex stuff, you may consider
 * not using this and call Vuforia directly so you can customize other stuff. Alternatively, you can also call
 * the getLocalizer method to get the Vuforia localizer object and use it to call Vuforia directly. This class
 * provides methods to simplify getting robot location from trackable images.
 */
public class FtcVuforia implements TrcVideoSource<Mat>
{
    /**
     * This class contains information about a trackable target.
     */
    public static class TargetInfo
    {
        public final int index;
        public final String name;
        public final boolean isObjectTarget;
        public final OpenGLMatrix locationOnField;
        public VuforiaTrackables targetList = null;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param index specifies the target index in the trackables list.
         * @param name specifies the target name.
         * @param isObjectTarget specifies true if target is an object, false if target is an image.
         * @param locationOnField specifies the target location on field.
         */
        public TargetInfo(int index, String name, boolean isObjectTarget, OpenGLMatrix locationOnField)
        {
            this.index = index;
            this.name = name;
            this.isObjectTarget = isObjectTarget;
            this.locationOnField = locationOnField;
        }   //TargetInfo

        private void setTargetList(VuforiaTrackables targetList)
        {
            this.targetList = targetList;
        }   //setTargetList

    }   //class TargetInfo

    private CameraName cameraName;
    private VuforiaLocalizer.CameraDirection cameraDir;
    private VuforiaLocalizer localizer;
    private ArrayList<VuforiaTrackables> targetLists = new ArrayList<>();
    private HashMap<String, TargetInfo> targetMap = new HashMap<>();
    private int numImageTargets = 0;
    private int numObjectTargets = 0;
    private int imageWidth = 0;
    private int imageHeight = 0;

    /**
     * Constructor: Create an instance of this object. It initializes Vuforia with the specified target images and
     * other parameters.
     *
     * @param vuforiaParams specifies the Vuforia parameters.
     */
    public FtcVuforia(VuforiaLocalizer.Parameters vuforiaParams)
    {
        if (vuforiaParams.cameraName != null)
        {
            this.cameraName = vuforiaParams.cameraName;
        }
        else if (vuforiaParams.cameraDirection != null)
        {
            this.cameraDir = vuforiaParams.cameraDirection;
        }
        else
        {
            throw new IllegalArgumentException(
                "Must specify either the camera direction on the phone or the USB camera name");
        }

        localizer = ClassFactory.getInstance().createVuforia(vuforiaParams);
    }   //FtcVuforia

    /**
     * This method returns the Vuforia Localizer.
     *
     * @return vuforia localizer.
     */
    public VuforiaLocalizer getLocalizer()
    {
        return localizer;
    }   //getLocalizer

    public void addTargetList(String trackablesFile, TargetInfo[] targets, OpenGLMatrix cameraLocationOnRobot)
    {
        VuforiaTrackables targetList = localizer.loadTrackablesFromAsset(trackablesFile);

        targetLists.add(targetList);
        for (TargetInfo targetInfo: targets)
        {
            VuforiaTrackable target = targetList.get(targetInfo.index);

            target.setName(targetInfo.name);
            targetInfo.setTargetList(targetList);

            if (targetInfo.locationOnField != null)
            {
                target.setLocation(targetInfo.locationOnField);
            }

            if (cameraLocationOnRobot != null)
            {
                if (cameraName != null)
                {
                    //
                    // USB Webcam.
                    //
                    ((VuforiaTrackableDefaultListener) target.getListener()).setCameraLocationOnRobot(
                        cameraName, cameraLocationOnRobot);
                }
                else
                {
                    //
                    // Phone camera.
                    //
                    ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(
                        cameraLocationOnRobot, cameraDir);
                }
            }

            targetMap.put(targetInfo.name, targetInfo);
            if (targetInfo.isObjectTarget)
            {
                numObjectTargets++;
            }
            else
            {
                numImageTargets++;
            }
        }

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, numImageTargets);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, numObjectTargets);
    }   //addTargetList

    /**
     * This method enables/disables target tracking.
     *
     * @param enabled specifies true to enable target tracking, false otherwise.
     */
    public void setTrackingEnabled(boolean enabled)
    {
        for (VuforiaTrackables targetList: targetLists)
        {
            if (enabled)
            {
                targetList.activate();
            }
            else
            {
                targetList.deactivate();
            }
        }
    }   //setTrackingEnabled

    /**
     * This method creates a location matrix that can be used to relocate an object to its final location by rotating
     * and translating the object from the origin of the field. It is doing the operation in the order of the
     * parameters. In other words, it will first rotate the object on the X-axis, then rotate on the Y-axis, then
     * rotate on the Z-axis, then translate on the X-axis, then translate on the Y-axis and finally translate on the
     * Z-axis.
     *
     * @param rotateX specifies rotation on the X-axis.
     * @param rotateY specifies rotation on the Y-axis.
     * @param rotateZ specifies rotation on the Z-axis.
     * @param translateX specifies translation on the X-axis.
     * @param translateY specifies translation on the Y-axis.
     * @param translateZ specifies translation on the Z-axis.
     * @return returns the location matrix.
     */
    public OpenGLMatrix locationMatrix(
            float rotateX, float rotateY, float rotateZ, float translateX, float translateY, float translateZ)
    {
        return OpenGLMatrix.translation(translateX, translateY, translateZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, rotateX, rotateY, rotateZ));
    }   //locationMatrix

    /**
     * This method returns the number of trackable image targets.
     *
     * @return number of trackable image targets.
     */
    public int getNumImageTargets()
    {
        return numImageTargets;
    }   //getNumImageTargets

    /**
     * This method returns the number of trackable object targets.
     *
     * @return number of trackable object targets.
     */
    public int getNumObjectTargets()
    {
        return numObjectTargets;
    }   //getNumObjectTargets

    /**
     * This method returns the target object with the specified index in the target list.
     *
     * @param name specifies the target name.
     * @return target.
     */
    public VuforiaTrackable getTarget(String name)
    {
        VuforiaTrackable target = null;
        TargetInfo targetInfo = targetMap.get(name);

        if (targetInfo != null)
        {
            target = targetInfo.targetList.get(targetInfo.index);
        }

        return target;
    }   //getTarget

    /**
     * This method determines if the target is visible.
     *
     * @param target specifies the target object.
     * @return true if the target is in view, false otherwise.
     */
    public boolean isTargetVisible(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.isVisible();
    }   //isTargetVisible

    /**
     * This method returns the position matrix of the specified target.
     *
     * @param target specifies the target to get the position matrix.
     * @return position matrix of the specified target.
     */
    public OpenGLMatrix getTargetPose(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getPose();
    }   //getTargetPose

    /**
     * This method determines the robot location by the given target.
     *
     * @param target specifies the target to be used to determine robot location.
     * @return robot location matrix.
     */
    public OpenGLMatrix getRobotLocation(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getRobotLocation();
    }   //getRobotLocation

    /**
     * This method configures Vuforia to capture video frames of the given format.
     *
     * @param imageWidth specifies the image width to capture.
     * @param imageHeight specifies the image height to capture.
     * @param queueCapacity specifies the frame queue capacity.
     */
    public void configVideoSource(int imageWidth, int imageHeight, int queueCapacity)
    {
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        localizer.setFrameQueueCapacity(queueCapacity);
    }   //configVideoSource

    /**
     * This methods turns the phone flash light ON or OFF.
     *
     * @param enabled specifies true to turn flashlight on, false to turn off.
     */
    public void setFlashlightEnabled(boolean enabled)
    {
        CameraDevice.getInstance().setFlashTorchMode(enabled);
    }   //setFlashlightEnabled

    //
    // Implements TrcVideoSource interface.
    //

    /**
     * This method gets a frame from the frame queue and returns the image that matches the format specified by the
     * configVideoSource method.
     *
     * @param frame specifies the frame object to hold image.
     * @return true if success, false otherwise.
     */
    @Override
    public boolean getFrame(Mat frame)
    {
        boolean success = false;

        try
        {
            VuforiaLocalizer.CloseableFrame closeableFrame = localizer.getFrameQueue().take();
            long numImages = closeableFrame.getNumImages();

            for (int i = 0; i < numImages; i++)
            {
                Image image = closeableFrame.getImage(i);

                if (image.getFormat() == PIXEL_FORMAT.RGB565 &&
                    (imageWidth == 0 || image.getWidth() == imageWidth) &&
                    (imageHeight == 0 || image.getHeight() == imageHeight))
                {
                    Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(image.getPixels());
                    Utils.bitmapToMat(bm, frame);
                    break;
                }
            }

            closeableFrame.close();
            success = true;
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        return success;
    }   //getFrame

    /**
     * This method draws the given image frame to the display surface.
     *
     * @param frame specifies the image frame to be displayed.
     */
    @Override
    public void putFrame(Mat frame)
    {
        // TODO: figure out how to render frame back to Vuforia.
    }   //putFrame

}   //class FtcVuforia
