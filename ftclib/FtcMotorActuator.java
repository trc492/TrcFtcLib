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
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcRobotBattery;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor or a continuous
 * rotation servo, optionally a lower limit switch, an upper limit switch and an encoder. It creates all the necessary
 * components for a PID controlled actuator which could include a software PID controller.
 */
public class FtcMotorActuator
{
    /**
     * This class contains all the parameters related to the actuator.
     */
    public static class Params
    {
        public boolean motorInverted = false;
        public boolean hasSlaveMotor = false;
        public boolean slaveMotorInverted = false;
        public boolean hasLowerLimitSwitch = false;
        public boolean lowerLimitSwitchInverted = false;
        public boolean hasUpperLimitSwitch = false;
        public boolean upperLimitSwitchInverted = false;
        public boolean hasExternalEncoder = false;
        public boolean encoderInverted = false;
        public boolean encoderAbsolute = false;
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
        public Params setMotorInverted(boolean inverted)
        {
            motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This methods sets the slave motor if there is one and also sets its direction.
         *
         * @param hasSlaveMotor specifies true if there is a slave motor, false otherwise.
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setSlaveMotor(boolean hasSlaveMotor, boolean inverted)
        {
            this.hasSlaveMotor = hasSlaveMotor;
            this.slaveMotorInverted = inverted;
            return this;
        }   //setSlaveMotor

        /**
         * This method sets the lower limit switch properties.
         *
         * @param hasLimitSwitch specifies true if there is a lower limit switch, false otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(boolean hasLimitSwitch, boolean inverted)
        {
            hasLowerLimitSwitch = hasLimitSwitch;
            lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch properties.
         *
         * @param hasLimitSwitch specifies true if there is an upper limit switch, false otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(boolean hasLimitSwitch, boolean inverted)
        {
            hasUpperLimitSwitch = hasLimitSwitch;
            upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitch

        /**
         * This method sets whether the actuator has an external encoder.
         *
         * @param hasEncoder specifies true if there is an external encoder, false otherwise.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @param absolute specifies true if the encoder is absolute, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(boolean hasEncoder, boolean inverted, boolean absolute)
        {
            hasExternalEncoder = hasEncoder;
            encoderInverted = inverted;
            encoderAbsolute = absolute;
            return this;
        }   //setExternalEncoder

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @return this object for chaining.
         */
        public Params setPositionScaleAndOffset(double scale, double offset)
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
        public Params setPositionPresets(double tolerance, double... posPresets)
        {
            positionPresets = posPresets;
            positionPresetTolerance = tolerance;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "motorInverted=%s,hasSlave=%s,slaveInverted=%s,hasLowerLimit=%s,lowerLimitInverted=%s," +
                "hasUpperLimit=%s,upperLimitInverted=%s,hasEncoder=%s,encoderInverted=%s, encoderAbs=%s" +
                "scale=%f,offset=%f,presetTolerance=%f,presets=%s",
                motorInverted, hasSlaveMotor, slaveMotorInverted, hasLowerLimitSwitch, lowerLimitSwitchInverted,
                hasUpperLimitSwitch, upperLimitSwitchInverted, hasExternalEncoder, encoderInverted, encoderAbsolute,
                positionScale, positionOffset, positionPresetTolerance, Arrays.toString(positionPresets));
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final TrcMotor actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param isCRServo specifies true if motor is a continuous rotation servo, false if it is a DC Motor.
     * @param params specifies the parameters to set up the actuator.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     * @param tracePidInfo specifies true to debug PID, ignore if tracer is null.
     * @param battery specifies the battery object for tracing battery level, ignore if tracer is null.
     */
    public FtcMotorActuator(
        String instanceName, boolean isCRServo, Params params, TrcDbgTrace tracer, boolean tracePidInfo,
        TrcRobotBattery battery)
    {
        FtcDigitalInput lowerLimitSwitch =
            params.hasLowerLimitSwitch? new FtcDigitalInput(instanceName + ".lowerLimit"): null;
        FtcDigitalInput upperLimitSwitch =
            params.hasUpperLimitSwitch? new FtcDigitalInput(instanceName + ".upperLimit"): null;
        FtcAnalogEncoder encoder =
            params.hasExternalEncoder? new FtcAnalogEncoder(instanceName + ".encoder"): null;

        if (encoder != null)
        {
            encoder.setInverted(params.encoderInverted);
            if (params.encoderAbsolute)
            {
                // Enable cartesian converter for absolute encoder.
                encoder.setEnabled(true);
            }
        }

        this.instanceName = instanceName;
        actuator =
            isCRServo? new FtcCRServo(instanceName + ".servo", lowerLimitSwitch, upperLimitSwitch, encoder):
                       new FtcDcMotor(instanceName + ".motor", lowerLimitSwitch, upperLimitSwitch, encoder);

        if (params.hasSlaveMotor)
        {
            TrcMotor slave =
                isCRServo? new FtcCRServo(instanceName + ".slaveServo"): new FtcDcMotor(instanceName + ".slaveMotor");
            slave.setMotorInverted(params.slaveMotorInverted);
            slave.follow(actuator);
        }

        if (lowerLimitSwitch != null)
        {
            actuator.enableLowerLimitSwitch(params.lowerLimitSwitchInverted);
        }

        if (upperLimitSwitch != null)
        {
            actuator.enableUpperLimitSwitch(params.upperLimitSwitchInverted);
        }

        if (!isCRServo)
        {
            actuator.setBrakeModeEnabled(true);
        }

        actuator.setOdometryEnabled(true, true, true);
        actuator.setMotorInverted(params.motorInverted);
        actuator.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        actuator.setPositionSensorScaleAndOffset(params.positionScale, params.positionOffset);
        actuator.setPosPresets(params.positionPresetTolerance, params.positionPresets);
        actuator.setMsgTracer(tracer, tracePidInfo, battery);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param isCRServo specifies true if motor is a continuous rotation servo, false if it is a DC Motor.
     * @param params specifies the parameters to set up the actuator.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     * @param tracePidInfo specifies true to debug PID, ignore if tracer is null.
     */
    public FtcMotorActuator(
        String instanceName, boolean isCRServo, Params params, TrcDbgTrace tracer, boolean tracePidInfo)
    {
        this(instanceName, isCRServo, params, tracer, tracePidInfo, null);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     * @param tracer specifies the tracer for debug tracing, can be null if not provided.
     * @param tracePidInfo specifies true to debug PID, ignore if tracer is null.
     */
    public FtcMotorActuator(String instanceName, Params params, TrcDbgTrace tracer, boolean tracePidInfo)
    {
        this(instanceName, false, params, tracer, tracePidInfo, null);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FtcMotorActuator(String instanceName, Params params)
    {
        this(instanceName, false, params, null, false, null);
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
     * This method returns the actuator object.
     *
     * @return actuator object.
     */
    public TrcMotor getActuator()
    {
        return actuator;
    }   //getActuator

}   //class FtcMotorActuator
