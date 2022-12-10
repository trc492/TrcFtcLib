/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Locale;

import TrcCommonLib.trclib.TrcPidActuator;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor, a lower limit
 * switch and an optional upper limit switch. It creates all the necessary components for a PID controlled actuator
 * which includes a PID controller and a PID controlled actuator.
 */
public class FtcMotorActuator
{
    /**
     * This class contains all the parameters related to the actuator motor.
     */
    public static class MotorParams
    {
        public boolean motorInverted;
        public boolean hasLowerLimitSwitch;
        public boolean lowerLimitInverted;
        public boolean hasUpperLimitSwitch;
        public boolean upperLimitInverted;
        public boolean lowerLimitZeroCalibrateOnly;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param motorInverted specifies true if actuator motor direction is inverted, false otherwise.
         * @param hasLowerLimitSwitch specifies true if actuator has a lower limit switch, false otherwise.
         * @param lowerLimitInverted specifies true if lower limit switch is inverted, false otherwise.
         * @param hasUpperLimitSwitch specifies true if actuator has an upper limit switch, false otherwise.
         * @param upperLimitInverted specifies true if upper limit switch is inverted, false otherwise.
         * @param lowerLimitZeroCalibrateOnly specifies true if lower limit switch is for zero calibration only.
         */
        public MotorParams(
            boolean motorInverted, boolean hasLowerLimitSwitch, boolean lowerLimitInverted,
            boolean hasUpperLimitSwitch, boolean upperLimitInverted, boolean lowerLimitZeroCalibrateOnly)
        {
            this.motorInverted = motorInverted;
            this.hasLowerLimitSwitch = hasLowerLimitSwitch;
            this.lowerLimitInverted = lowerLimitInverted;
            this.hasUpperLimitSwitch = hasUpperLimitSwitch;
            this.upperLimitInverted = upperLimitInverted;
            this.lowerLimitZeroCalibrateOnly = lowerLimitZeroCalibrateOnly;
        }   //MotorParams

        /**
         * Constructor: Create an instance of the object.
         *
         * @param motorInverted specifies true if actuator motor direction is inverted, false otherwise.
         * @param hasLowerLimitSwitch specifies true if actuator has a lower limit switch, false otherwise.
         * @param lowerLimitInverted specifies true if lower limit switch is inverted, false otherwise.
         * @param hasUpperLimitSwitch specifies true if actuator has an upper limit switch, false otherwise.
         * @param upperLimitInverted specifies true if upper limit switch is inverted, false otherwise.
         */
        public MotorParams(
            boolean motorInverted, boolean hasLowerLimitSwitch, boolean lowerLimitInverted,
            boolean hasUpperLimitSwitch, boolean upperLimitInverted)
        {
            this(motorInverted, hasLowerLimitSwitch, lowerLimitInverted, hasUpperLimitSwitch, upperLimitInverted,
                 false);
        }   //MotorParams

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
                "motorInverted=%s,hasLowerLimitSW=%s,lowerLimitInverted=%s,hasUpperLimitSW=%s,upperLimitInverted=%s," +
                "lowerLimitZeroCalOnly=%s",
                motorInverted, hasLowerLimitSwitch, lowerLimitInverted, hasUpperLimitSwitch, upperLimitInverted,
                lowerLimitZeroCalibrateOnly);
        }   //toString

    }   //class MotorParams

    private final TrcPidActuator pidActuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    public FtcMotorActuator(String instanceName, MotorParams motorParams, TrcPidActuator.Parameters actuatorParams)
    {
        FtcDigitalInput lowerLimitSwitch =
            motorParams.hasLowerLimitSwitch?
                new FtcDigitalInput(instanceName + ".lowerLimit", motorParams.lowerLimitInverted): null;
        FtcDigitalInput upperLimitSwitch =
            motorParams.hasUpperLimitSwitch?
                new FtcDigitalInput(instanceName + ".upperLimit", motorParams.upperLimitInverted): null;

        FtcDcMotor actuatorMotor = new FtcDcMotor(
            instanceName + ".motor",
            !motorParams.lowerLimitZeroCalibrateOnly? lowerLimitSwitch: null, upperLimitSwitch);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.setOdometryEnabled(true, true, true);
        actuatorMotor.setInverted(motorParams.motorInverted);

        pidActuator = new TrcPidActuator(
            instanceName, actuatorMotor, lowerLimitSwitch, upperLimitSwitch, actuatorParams);
    }   //FtcMotorActuator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return pidActuator.toString();
    }   //toString

    /**
     * This method returns the PID actuator object.
     *
     * @return PID actuator object.
     */
    public TrcPidActuator getPidActuator()
    {
        return pidActuator;
    }   //getPidActuator

}   //class FtcMotorActuator

