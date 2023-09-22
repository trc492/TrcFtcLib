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
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor, a lower limit
 * switch and an optional upper limit switch. It creates all the necessary components for a PID controlled actuator
 * which could include a software PID controller.
 */
public class FtcMotorActuator
{
    /**
     * This class contains all the parameters related to the actuator motor.
     */
    public static class MotorParams
    {
        public boolean motorInverted = false;
        public boolean lowerLimitSwitchEnabled = false;
        public boolean lowerLimitSwitchInverted = false;
        public boolean upperLimitSwitchEnabled = false;
        public boolean upperLimitSwitchInverted = false;
        public double positionScale = 1.0;
        public double positionOffset = 0.0;
        public double[] positionPresets = null;
        public double positionPresetTolerance = 0.0;

        /**
         * This methods sets the motor direction.
         *
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public MotorParams setMotorInverted(boolean inverted)
        {
            motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This method sets the lower limit switch properties.
         *
         * @param enabled specifies true if there is a lower limit switch, false if none.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public MotorParams setLowerLimitSwitchEnabled(boolean enabled, boolean inverted)
        {
            lowerLimitSwitchEnabled = enabled;
            lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitchEnabled

        /**
         * This method sets the upper limit switch properties.
         *
         * @param enabled specifies true if there is an upper limit switch, false if none.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public MotorParams setUpperLimitSwitchEnabled(boolean enabled, boolean inverted)
        {
            upperLimitSwitchEnabled = enabled;
            upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitchEnabled

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @return this object for chaining.
         */
        public MotorParams setPositionScaleAndOffset(double scale, double offset)
        {
            positionScale = scale;
            positionOffset = offset;
            return this;
        }   //setPositionScaleAndOffset

        /**
         * This method sets an array of preset positions for the motor actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public MotorParams setPositionPresets(double tolerance, double... posPresets)
        {
            positionPresets = posPresets;
            positionPresetTolerance = tolerance;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the motorParams info.
         *
         * @return string format of the motor param info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "motorInverted=%s,lowerLimitEnabled=%s,lowerLimitInverted=%s," +
                "upperLimitEnabled=%s,upperLimitInverted=%s,scale=%f,offset=%f,presetTolerance=%f,presets=%s",
                motorInverted, lowerLimitSwitchEnabled, lowerLimitSwitchInverted,
                upperLimitSwitchEnabled, upperLimitSwitchInverted, positionScale, positionOffset,
                positionPresetTolerance, Arrays.toString(positionPresets));
        }   //toString

    }   //class MotorParams

    private final String instanceName;
    private final FtcDcMotor actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     * @param tracePidInfo specifies true to debug PID, ignore if tracer is null.
     * @param battery specifies the battery object for tracing battery level, ignore if tracer is null.
     */
    public FtcMotorActuator(
        String instanceName, MotorParams motorParams, TrcDbgTrace tracer, boolean tracePidInfo, TrcRobotBattery battery)
    {
        FtcDigitalInput lowerLimitSwitch =
            motorParams.lowerLimitSwitchEnabled? new FtcDigitalInput(instanceName + ".lowerLimit"): null;
        FtcDigitalInput upperLimitSwitch =
            motorParams.upperLimitSwitchEnabled? new FtcDigitalInput(instanceName + ".upperLimit"): null;

        this.instanceName = instanceName;
        if (lowerLimitSwitch != null)
        {
            lowerLimitSwitch.setInverted(motorParams.lowerLimitSwitchInverted);
        }

        if (upperLimitSwitch != null)
        {
            upperLimitSwitch.setInverted(motorParams.upperLimitSwitchInverted);
        }

        actuator = new FtcDcMotor(instanceName + ".motor", lowerLimitSwitch, upperLimitSwitch);
        actuator.setBrakeModeEnabled(true);
        actuator.setOdometryEnabled(true, true, true);
        actuator.setMotorInverted(motorParams.motorInverted);
        actuator.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        actuator.setPositionSensorScaleAndOffset(motorParams.positionScale, motorParams.positionOffset);
        actuator.setPosPresets(motorParams.positionPresetTolerance, motorParams.positionPresets);
        actuator.setMsgTracer(tracer, tracePidInfo, battery);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     * @param tracePidInfo specifies true to debug PID, ignore if tracer is null.
     */
    public FtcMotorActuator(String instanceName, MotorParams motorParams, TrcDbgTrace tracer, boolean tracePidInfo)
    {
        this(instanceName, motorParams, tracer, tracePidInfo, null);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motorParams specifies the parameters to set up the actuator motor.
     */
    public FtcMotorActuator(String instanceName, MotorParams motorParams)
    {
        this(instanceName, motorParams, null, false, null);
    }   //FtcMotorActuator

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
    public FtcDcMotor getMotor()
    {
        return actuator;
    }   //getMotor

}   //class FtcMotorActuator

