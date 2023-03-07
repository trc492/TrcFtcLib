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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import TrcCommonLib.trclib.TrcAnalogInput;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements the generic DC Motor Controller extending TrcMotor. It provides implementation of the
 * TrcMotorController interface in TrcMotor. It supports limit switches. When this class is constructed with limit
 * switches, setPower will respect them and will not move the motor into the direction where the limit switch is
 * activated.
 */
public class FtcDcMotor extends TrcMotor
{
    private static final double DEF_RESET_TIMEOUT = 0.1;
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    private final String instanceName;
    private final TrcDigitalInput revLimitSwitch;
    private final TrcDigitalInput fwdLimitSwitch;
    private final TrcAnalogInput analogSensor;
    public final DcMotorEx motor;
    private final VoltageSensor voltageSensor;

    private DcMotor.RunMode runMode;
    private boolean posSensorInverted = false;
    private Double prevMotorValue = null;
    private double batteryNominalVoltage = 0.0;

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
        this.instanceName = instanceName;
        this.revLimitSwitch = revLimitSwitch;
        this.fwdLimitSwitch = fwdLimitSwitch;
        this.analogSensor = analogSensor;
        motor = hardwareMap.get(DcMotorEx.class, instanceName);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        runMode = motor.getMode();
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
    // Overrides TrcMotor methods.
    //

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
            globalTracer.traceInfo(funcName, "maxVel=%f,pidCoef=%s", maxVelocity, pidCoefficients);
        }

        this.maxMotorVelocity = maxVelocity;
        if (pidCoefficients != null)
        {
            motor.setVelocityPIDFCoefficients(
                pidCoefficients.kP, pidCoefficients.kI, pidCoefficients.kD, pidCoefficients.kF);
        }
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        maxMotorVelocity = 0.0;
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }   //disableVelocityMode

    //
    // Implements TrcMotorController interface.
    //

    /**
     * This method is used to check if the motor controller supports close loop control natively.
     *
     * @return true if motor controller supports close loop control, false otherwise.
     */
    @Override
    public boolean supportCloseLoopControl()
    {
        return true;
    }   //supportCloseloopControl

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        motor.resetDeviceConfigurationForOpMode();
    }   //resetFactoryDefault

    /**
     * This method checks if the motor controller is connected to the robot. Note that this does NOT guarantee the
     * connection status of the motor to the motor controller. If detecting the motor presence is impossible (i.e. the
     * motor controller is connected via PWM) this method will always return true.
     *
     * @return true if the motor is connected or if it's impossible to know, false otherwise.
     */
    @Override
    public boolean isConnected()
    {
        return motor.isMotorEnabled();
    }   //isConnected

    /**
     * This method sets this motor to follow another motor.
     *
     * @param motor specifies the motor to follow.
     */
    @Override
    public void followMotor(TrcMotor motor)
    {
        motor.addFollowingMotor(this);
    }   //followMotor

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return voltageSensor.getVoltage();
    }   //getBusVoltage

    /**
     * This method enables voltage compensation so that it will maintain the motor output regardless of battery
     * voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery.
     */
    @Override
    public void enableVoltageCompensation(double batteryNominalVoltage)
    {
        this.batteryNominalVoltage = batteryNominalVoltage;
    }   //enableVoltageCompensation

    /**
     * This method disables voltage compensation.
     */
    @Override
    public void disableVoltageCompensation()
    {
        this.batteryNominalVoltage = 0.0;
    }   //disableVoltageCompensation

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    @Override
    public boolean isVoltageCompensationEnabled()
    {
        return batteryNominalVoltage > 0.0;
    }   //isVoltageCompensationEnabled

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        motor.setZeroPowerBehavior(enabled? DcMotorEx.ZeroPowerBehavior.BRAKE: DcMotorEx.ZeroPowerBehavior.FLOAT);
    }   //setBrakeModeEnabled

    /**
     * This method sets the PID coefficients for the motor's position PID controller. For FTC motors, only kP makes
     * sense because of the double layer architecture (i.e. position PID is used in conjunction with velocity PID).
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        motor.setPositionPIDFCoefficients(pidCoeff.kP);
    }   //setPidCoefficients

    /**
     * This method sets the PID coefficients of the motor's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        motor.setVelocityPIDFCoefficients(pidCoeff.kP, pidCoeff.kI, pidCoeff.kD, pidCoeff.kF);
    }   //setVelocityPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getPidCoefficients()
    {
        PIDFCoefficients pidfCoeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        return new TrcPidController.PidCoefficients(pidfCoeffs.p, pidfCoeffs.i, pidfCoeffs.d, pidfCoeffs.f);
    }   //getPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    public TrcPidController.PidCoefficients getVelocityPidCoefficients()
    {
        PIDFCoefficients pidfCoeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        return new TrcPidController.PidCoefficients(pidfCoeffs.p, pidfCoeffs.i, pidfCoeffs.d, pidfCoeffs.f);
    }   //getVelocityPidCoefficients

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    @Override
    public void setRevLimitSwitchInverted(boolean inverted)
    {
        if (revLimitSwitch != null)
        {
            revLimitSwitch.setInverted(inverted);
        }
    }   //setRevLimitSwitchInverted

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    @Override
    public void setFwdLimitSwitchInverted(boolean inverted)
    {
        if (fwdLimitSwitch != null)
        {
            fwdLimitSwitch.setInverted(inverted);
        }
    }   //setFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        return revLimitSwitch != null && revLimitSwitch.isActive();
    }   //isRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        return fwdLimitSwitch != null && fwdLimitSwitch.isActive();
    }   //isFwdLimitSwitchActive

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        motor.setDirection(inverted? DcMotorEx.Direction.REVERSE: DcMotorEx.Direction.FORWARD);
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.getDirection() == DcMotorEx.Direction.REVERSE;
    }   //isMotorInverted

    /**
     * This method stops the motor regardless of what control mode the motor is on.
     */
    @Override
    public void stopMotor()
    {
        setMotorPower(0.0);
    }   //stopMotor

    /**
     * This method sets the raw motor power.
     *
     * @param value specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double value)
    {
        final String funcName = "setMotorPower";
        // Allow this only if we are not in close loop control mode.
        if (runMode == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            double origValue = value;

            if (batteryNominalVoltage != 0.0)
            {
                value *= batteryNominalVoltage / voltageSensor.getVoltage();
                value = TrcUtil.clipRange(value);
            }

            if (value > 0.0 && fwdLimitSwitch != null && fwdLimitSwitch.isActive() ||
                value < 0.0 && revLimitSwitch != null && revLimitSwitch.isActive())
            {
                value = 0.0;
            }

            if (prevMotorValue == null || prevMotorValue != value)
            {
                motor.setPower(value);
                prevMotorValue = value;
            }

            if (debugEnabled)
            {
                globalTracer.traceInfo(funcName, "origValue=%f,value=%f", origValue, value);
            }
        }
    }   //setMotorPower

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    @Override
    public double getMotorPower()
    {
        return motor.getPower();
    }   //getMotorPower

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        posSensorInverted = inverted;
    }   //setPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isPositionSensorInverted()
    {
        return posSensorInverted;
    }   //isPositionSensorInverted

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param position specifies the motor position in raw sensor units (encoder counts).
     */
    @Override
    public void setMotorPosition(double position)
    {
        // Allow this only if we are in close loop position control mode.
        if (runMode == DcMotor.RunMode.RUN_TO_POSITION)
        {
            motor.setTargetPosition((int) position);
        }
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in raw sensor units.
     */
    @Override
    public double getMotorPosition()
    {
        final String funcName = "getMotorPosition";
        double currPos = analogSensor == null?
            motor.getCurrentPosition(): analogSensor.getRawData(0, TrcAnalogInput.DataType.INPUT_DATA).value;

        if (posSensorInverted)
        {
            currPos *= -1.0;
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "pos=%d", currPos);
        }

        return currPos;
    }   //getMotorPosition

    /**
     * This method resets the motor position sensor, typically an encoder. This is a synchronous call and will take
     * time. It should only be called at robot init time.
     *
     * @param timeout specifies a timeout period in seconds.
     */
    public void resetMotorPosition(double timeout)
    {
        final String funcName = "resetMotorPosition";
        //
        // Resetting the encoder is done by setting the motor controller mode. This is a long operation and has side
        // effect of disabling the motor controller unless you do another setMode to re-enable it. Therefore,
        // resetMotorPosition is a synchronous call. This should only be called at robotInit time. For other times,
        // you should call software resetPosition instead.
        //
        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "[%.3f] Before resetting %s: enc=%d",
                TrcTimer.getModeElapsedTime(), instanceName, motor.getCurrentPosition());
        }
        DcMotor.RunMode prevMotorMode = motor.getMode();
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        prevMotorValue = null;

        double expiredTime = TrcTimer.getCurrentTime() + timeout;
        int motorPos = 0;
        while (TrcTimer.getCurrentTime() < expiredTime)
        {
            FtcOpMode.getInstance().clearBulkCacheInManualMode();
            motorPos = motor.getCurrentPosition();
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] Waiting for %s to reset: enc=%d",
                    TrcTimer.getModeElapsedTime(), instanceName, motorPos);
            }

            if (motorPos != 0)
            {
                Thread.yield();
            }
            else
            {
                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "[%.3f] Reset %s success!", TrcTimer.getModeElapsedTime(), instanceName);
                }
                break;
            }
        }

        if (motorPos != 0)
        {
            globalTracer.traceWarn(funcName, "motor %s encoder reset timed out (enc=%d).", instanceName, motorPos);
        }

        // Restore previous motor mode.
        motor.setMode(prevMotorMode);

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "timeout=%.3f,pos=%d", timeout, motorPos);
        }
    }   //resetMotorPosition

    /**
     * This method resets the motor position sensor, typically an encoder. This is a synchronous call and will take
     * time. It should only be called at robot init time.
     */
    @Override
    public void resetMotorPosition()
    {
        resetMotorPosition(DEF_RESET_TIMEOUT);
    }   //resetMotorPosition

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param value specifies the motor velocity in raw sensor units per second (encoder counts per sec).
     */
    @Override
    public void setMotorVelocity(double value)
    {
        final String funcName = "setMotorVelocity";
        // Allow this only if we are in close loop velocity control mode.
        if ( runMode == DcMotor.RunMode.RUN_USING_ENCODER)
        {
            double origValue = value;

            if (value > 0.0 && fwdLimitSwitch != null && fwdLimitSwitch.isActive() ||
                value < 0.0 && revLimitSwitch != null && revLimitSwitch.isActive())
            {
                value = 0.0;
            }

            if (prevMotorValue == null || prevMotorValue != value)
            {
                motor.setVelocity(value);
                prevMotorValue = value;
            }

            if (debugEnabled)
            {
                globalTracer.traceInfo(funcName, "origValue=%f,value=%f", origValue, value);
            }
        }
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in raw sensor units per sec (counts per sec).
     */
    @Override
    public double getMotorVelocity()
    {
        return motor.getVelocity();
    }   //getMotorVelocity

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        throw new UnsupportedOperationException("Motor controller does not support current control mode.");
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        return motor.getCurrent(CurrentUnit.AMPS);
    }   //getMotorCurrent

    /**
     * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
     * By setting a non-default limits, it effectively limits the output power of the close loop control.
     *
     * @param revLimit specifies the percentage output limit of the reverse direction.
     * @param fwdLimit specifies the percentage output limit of the forward direction.
     */
    @Override
    public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
    {
        // Allow this only if we are in close loop position control mode.
        if (runMode == DcMotor.RunMode.RUN_TO_POSITION)
        {
            revLimit = Math.abs(revLimit);
            fwdLimit = Math.abs(fwdLimit);

            if (revLimit != fwdLimit)
            {
                throw new IllegalArgumentException("revLimit and fwdLimit must have the same magnitude.");
            }

            motor.setPower(fwdLimit);
        }
    }   //setCloseLoopOutputLimits

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent specifies threshold current in amperes to be exceeded before limiting occurs.
     *        If this value is less than currentLimit, then currentLimit is used as the threshold.
     * @param triggerThresholdTime specifies how long current must exceed threshold (seconds) before limiting occurs.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        throw new UnsupportedOperationException("Motor controller does not support setting current limit.");
    }   //setCurrentLimit

}   //class FtcDcMotor
