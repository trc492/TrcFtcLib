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

import TrcCommonLib.trclib.TrcPidActuator;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor, a lower limit
 * switch and an optional upper limit switch. It creates all the necessary components for a PID controlled actuator
 * which includes a PID controller and a PID controlled actuator.
 */
public class FtcMotorActuator
{
    private final TrcPidActuator pidActuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the PID actuator.
     */
    public FtcMotorActuator(String instanceName, TrcPidActuator.Parameters params)
    {
        FtcDigitalInput lowerLimitSwitch =
            params.hasLowerLimitSwitch?
                new FtcDigitalInput(instanceName + ".lowerLimit", params.lowerLimitInverted): null;
        FtcDigitalInput upperLimitSwitch =
            params.hasUpperLimitSwitch?
                new FtcDigitalInput(instanceName + ".upperLimit", params.upperLimitInverted): null;

        FtcDcMotor actuatorMotor = new FtcDcMotor(instanceName + ".motor", lowerLimitSwitch, upperLimitSwitch);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.setOdometryEnabled(true);
        actuatorMotor.setInverted(params.motorInverted);

        pidActuator = new TrcPidActuator(instanceName, actuatorMotor, lowerLimitSwitch, upperLimitSwitch, params);
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

