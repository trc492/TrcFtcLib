/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import TrcCommonLib.trclib.TrcAnalogInput;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;

/**
 * This class implements the generic DC Motor Controller extending TrcMotor. It provides implementation of the
 * abstract methods in TrcMotor. It supports limit switches. When this class is constructed with limit switches,
 * setPower will respect them and will not move the motor into the direction where the limit switch is activated.
 * It also provides a software encoder reset without switching the Modern Robotics motor controller mode which is
 * problematic.
 */
public class FtcDcMotor extends TrcMotor
{
    private static final String moduleName = "FtcDcMotor";

    private final String instanceName;
    private final TrcDigitalInput revLimitSwitch;
    private final TrcDigitalInput fwdLimitSwitch;
    private final TrcAnalogInput analogSensor;
    public final DcMotor motor;
    private int prevEncPos;
    private double prevMotorValue = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param revLimitSwitch specifies the reverse limit switch object.
     * @param fwdLimitSwitch specifies the forward limit switch object.
     * @param analogSensor specifies an analog position sensor instead of the motor encoder.
     */
    public FtcDcMotor(HardwareMap hardwareMap, String instanceName,
                      TrcDigitalInput revLimitSwitch, TrcDigitalInput fwdLimitSwitch, TrcAnalogInput analogSensor)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer? globalTracer:
                    new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.revLimitSwitch = revLimitSwitch;
        this.fwdLimitSwitch = fwdLimitSwitch;
        this.analogSensor = analogSensor;
        motor = hardwareMap.dcMotor.get(instanceName);
        prevEncPos = motor.getCurrentPosition();
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param revLimitSwitch specifies the reverse limit switch object.
     * @param fwdLimitSwitch specifies the forward limit switch object.
     * @param analogSensor specifies an analog position sensor instead of the motor encoder.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput revLimitSwitch, TrcDigitalInput fwdLimitSwitch,
                      TrcAnalogInput analogSensor)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, revLimitSwitch, fwdLimitSwitch, analogSensor);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param revLimitSwitch specifies the reverse limit switch object.
     * @param fwdLimitSwitch specifies the forward limit switch object.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput revLimitSwitch, TrcDigitalInput fwdLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, revLimitSwitch, fwdLimitSwitch, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param revLimitSwitch specifies the reverse limit switch object.
     */
    public FtcDcMotor(String instanceName, TrcDigitalInput revLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, revLimitSwitch, null, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDcMotor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, null, null, null);
    }   //FtcDcMotor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    //
    // Implements TrcMotor abstract methods.
    //

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        final String funcName = "setBrakeModeEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setZeroPowerBehavior(enabled? DcMotor.ZeroPowerBehavior.BRAKE: DcMotor.ZeroPowerBehavior.FLOAT);
    }   //setBrakeModeEnabled

    /**
     * This method sets the raw motor power.
     *
     * @param value specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public synchronized void setMotorPower(double value)
    {
        final String funcName = "setMotorPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", value);
        }

        if (value > 0.0 && fwdLimitSwitch != null && fwdLimitSwitch.isActive() ||
            value < 0.0 && revLimitSwitch != null && revLimitSwitch.isActive())
        {
            value = 0.0;
        }

        if (value != prevMotorValue)
        {
            motor.setPower(value);
            prevMotorValue = value;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", value);
        }
    }   //setMotorPower

    /**
     * This method stops the motor regardless of what control mode the motor is on.
     */
    public void stopMotor()
    {
        setMotorPower(0.0);
    }   //stopMotor

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getMotorPower()
    {
        final String funcName = "getMotorPower";
        double power = motor.getPower();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", power);
        }

        return power;
    }   //getMotorPower

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public synchronized void resetMotorPosition()
    {
        final String funcName = "resetMotorPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
        //
        // Modern Robotics motor controllers supports resetting encoders by setting the motor controller mode. This
        // is a long operation and has side effect of disabling the motor controller unless you do another setMode
        // to re-enable it. Therefore, resetPosition with hardware set to true is a synchronous call. This should
        // only be called at robotInit time. For other times, it should call resetPosition with hardware set to false
        // (software reset).
        //
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motor.getCurrentPosition() != 0.0)
        {
            Thread.yield();
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }   //resetMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public synchronized double getMotorPosition()
    {
        final String funcName = "getMotorPosition";
        double currPos = analogSensor == null?
                motor.getCurrentPosition(): analogSensor.getRawData(0, TrcAnalogInput.DataType.INPUT_DATA).value;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (analogSensor == null)
        {
            //
            // Somebody said if motor controller got disconnected, we may get a zero. Let's detect this and see if this
            // really happened.
            //
            if (currPos == 0.0 && Math.abs(prevEncPos) > 1000)
            {
                globalTracer.traceWarn(
                        funcName, "Detected possible motor controller disconnect for %s (prevEncPos=%d).",
                        instanceName, prevEncPos);
                currPos = prevEncPos;
            }
            else
            {
                prevEncPos = (int)currPos;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currPos);
        }

        return currPos;
    }   //getMotorPosition

    /**
     * This method returns the motor velocity from the platform dependent motor hardware. If the hardware does
     * not support velocity info, it should throw an UnsupportedOperationException.
     *
     * @return current motor velocity in ticks per second.
     */
    @Override
    public double getMotorVelocity()
    {
        final String funcName = "getMotorVelocity";
        double currVel = ((DcMotorEx)motor).getVelocity();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currVel);
        }

        return currVel;
    }   //getMotorVelocity

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        final String funcName = "isRevLimitSwitchActive";
        boolean isActive = revLimitSwitch != null && revLimitSwitch.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
        }

        return isActive;
    }   //isRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        final String funcName = "isFwdLimitSwitchActive";
        boolean isActive = fwdLimitSwitch != null && fwdLimitSwitch.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(isActive));
        }

        return isActive;
    }   //isFwdLimitSwitchActive

    /**
     * This method sets the motor velocity.
     *
     * @param value specifies the percentage of max motor velocity (range -1.0 to 1.0) to be set.
     */
    private synchronized void setMotorVelocity(double value)
    {
        final String funcName = "setMotorVelocity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", value);
        }

        if (value > 0.0 && fwdLimitSwitch != null && fwdLimitSwitch.isActive() ||
            value < 0.0 && revLimitSwitch != null && revLimitSwitch.isActive())
        {
            value = 0.0;
        }

        if (value != prevMotorValue)
        {
            ((DcMotorEx)motor).setVelocity(value*maxMotorVelocity);
            prevMotorValue = value;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", value);
        }
    }   //setMotorVelocity

    //
    // Overrides TrcMotor methods.
    //

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setDirection(inverted? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
    }   //setInverted

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        final String funcName = "isInverted";
        boolean inverted = motor.getDirection() == DcMotor.Direction.REVERSE;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
        }

        return inverted;
    }   //isInverted

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PID coefficients to use to for velocity control. If null, use default
     *                        PID coefficients built-in to the motor.
     */
    @Override
    public synchronized void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoefficients)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoef=%s",
                                maxVelocity, pidCoefficients);
        }

        this.maxMotorVelocity = maxVelocity;
        if (pidCoefficients != null)
        {
            ((DcMotorEx)motor).setVelocityPIDFCoefficients(
                pidCoefficients.kP, pidCoefficients.kI, pidCoefficients.kD, pidCoefficients.kF);
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //enableVelocityMode

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity specifies the maximum velocity the motor can run, in sensor units per second.
     */
    public void enableVelocityMode(double maxVelocity)
    {
        enableVelocityMode(maxVelocity, null);
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    @Override
    public synchronized void disableVelocityMode()
    {
        final String funcName = "disableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        maxMotorVelocity = 0.0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }   //disableVelocityMode

}   //class FtcDcMotor
