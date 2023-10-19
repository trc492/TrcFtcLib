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

import java.util.Arrays;
import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;

public class FtcServoActuator
{
    /**
     * This class contains all the parameters related to the actuator servo.
     */
    public static class ServoParams
    {
        public boolean servoInverted = false;
        public boolean hasServo2 = false;
        public boolean servo2Inverted = false;
        public double logicalPosMin = 0.0;
        public double logicalPosMax = 1.0;
        public double physicalPosMin = 0.0;
        public double physicalPosMax = 1.0;
        public double presetTolerance = 0.0;
        public double[] positionPresets = null;

        /**
         * This methods sets the servo direction.
         *
         * @param inverted specifies true to invert servo direction, false otherwise.
         * @return this object for chaining.
         */
        public ServoParams setServoInverted(boolean inverted)
        {
            servoInverted = inverted;
            return this;
        }   //setServoInverted

        /**
         * This methods sets if the actuator has a second servo and if the second servo is inverted.
         *
         * @param hasServo2 specifies true if the actuator has a second servo, false otherwise.
         * @param servo2Inverted specifies true if the second servo is inverted, false otherwise. Only applicable if
         *        hasServo2 is true.
         * @return this object for chaining.
         */
        public ServoParams setHasServo2(boolean hasServo2, boolean servo2Inverted)
        {
            this.hasServo2 = hasServo2;
            this.servo2Inverted = servo2Inverted;
            return this;
        }   //setHasServo2

        /**
         * This method sets the logical position range of the servo in the range of 0.0 to 1.0.
         *
         * @param minPos specifies the min logical position.
         * @param maxPos specifies the max logical position.
         * @return this object for chaining.
         */
        public ServoParams setLogicalPosRange(double minPos, double maxPos)
        {
            logicalPosMin = minPos;
            logicalPosMax = maxPos;
            return this;
        }   //setLogicalPosRange

        /**
         * This method sets the physical position range of the servo in real world physical unit.
         *
         * @param minPos specifies the min physical position.
         * @param maxPos specifies the max physical position.
         * @return this object for chaining.
         */
        public ServoParams setPhysicalPosRange(double minPos, double maxPos)
        {
            physicalPosMin = minPos;
            physicalPosMax = maxPos;
            return this;
        }   //setPhysicalPosRange

        /**
         * This method sets an array of preset positions for the servo actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public ServoParams setPositionPresets(double tolerance, double... posPresets)
        {
            presetTolerance = tolerance;
            positionPresets = posPresets;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the servoParams info.
         *
         * @return string format of the servo param info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "servoInverted=%s,hasServo2=%s,servo2Inverted=%s,logicalMin=%f,logicalMax=%f,phyMin=%f,phyMax=%f," +
                "presets=%s",
                servoInverted, hasServo2, servo2Inverted, logicalPosMin, logicalPosMax, physicalPosMin, physicalPosMax,
                Arrays.toString(positionPresets));
        }   //toString

    }   //class ServoParams

    private final String instanceName;
    private final FtcServo actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servoParams specifies the parameters to set up the actuator servo.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     */
    public FtcServoActuator(String instanceName, ServoParams servoParams, TrcDbgTrace tracer)
    {
        this.instanceName = instanceName;
        actuator = new FtcServo(instanceName + ".servo");
        actuator.setInverted(servoParams.servoInverted);
        actuator.setLogicalPosRange(servoParams.logicalPosMin, servoParams.logicalPosMax);
        actuator.setPhysicalPosRange(servoParams.physicalPosMin, servoParams.physicalPosMax);
        actuator.setPosPresets(servoParams.presetTolerance, servoParams.positionPresets);
        actuator.setMsgTracer(tracer);
        if (servoParams.hasServo2)
        {
            FtcServo servo2 = new FtcServo(instanceName + ".servo2");
            servo2.setInverted(servoParams.servo2Inverted);
            servo2.follow(actuator);
        }
    }   //FtcServoActuator

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
     * This method returns the actuator motor object.
     *
     * @return actuator motor object.
     */
    public FtcServo getServo()
    {
        return actuator;
    }   //getServo

}   //class FtcServoActuator
