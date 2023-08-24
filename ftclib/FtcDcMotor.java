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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements an FTC DC Motor Controller extending TrcMotor. It provides implementation of the
 * TrcMotorController interface. It supports limit switches and an external position sensor (e.g. encoder).
 * When this class is constructed with limit switches, all motor movements will respect them and will not move the
 * motor into the direction where the limit switch is activated.
 */
public class FtcDcMotor extends TrcMotor
{
    private static final double DEF_POS_RESET_TIMEOUT = 0.1;
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    public final DcMotorEx motor;
    private final VoltageSensor voltageSensor;
    private DcMotor.RunMode runMode;
    private double velocityPidTolerance = 0.0;
    private double velocityTarget = 0.0;
    private int positionTarget = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies an external lower limit switch, can be null if not provided.
     * @param upperLimitSwitch specifies an external upper limit switch, can be null if not provided.
     * @param encoder specifies an external position sensor overriding the motor controller one, can be null
     *        if not provided.
     */
    public FtcDcMotor(
        HardwareMap hardwareMap, String instanceName, TrcDigitalInput lowerLimitSwitch,
        TrcDigitalInput upperLimitSwitch, TrcEncoder encoder)
    {
        super(instanceName, lowerLimitSwitch, upperLimitSwitch, encoder);
        motor = hardwareMap.get(DcMotorEx.class, instanceName);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        runMode = motor.getMode();
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies an external lower limit switch, can be null if not provided.
     * @param upperLimitSwitch specifies an external upper limit switch, can be null if not provided.
     * @param encoder specifies an external position sensor overriding the motor controller one, can be null
     *        if not provided.
     */
    public FtcDcMotor(
        String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch, TrcEncoder encoder)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, upperLimitSwitch, encoder);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param lowerLimitSwitch specifies an external lower limit switch, can be null if not provided.
     * @param upperLimitSwitch specifies an external upper limit switch, can be null if not provided.
     */
    public FtcDcMotor(
        String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, upperLimitSwitch, null);
    }   //FtcDcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param encoder specifies an external position sensor overriding the motor controller one, can be null
     *        if not provided.
     */
    public FtcDcMotor(String instanceName, TrcEncoder encoder)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, null, null, encoder);
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

    //
    // Implements TrcMotorController interface.
    //

//    /**
//     * This method is used to check if the motor controller supports close loop control.
//     *
//     * @return true if motor controller supports close loop control, false otherwise.
//     */
//    @Override
//    public boolean supportNativeCloseLoopControl()
//    {
//        return true;
//    }   //supportNativeCloseloopControl

//    /**
//     * This method checks if the motor controller is connected to the robot.
//     *
//     * @return true if the motor is connected or if it's impossible to know, false otherwise.
//     */
//    @Override
//    public boolean isConnected()
//    {
//        return motor.isMotorEnabled();
//    }   //isConnected

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        motor.resetDeviceConfigurationForOpMode();
        runMode = motor.getMode();
    }   //resetFactoryDefault

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
        throw new UnsupportedOperationException("Motor controller does not support setCurrentLimit.");
    }   //setCurrentLimit

//    /**
//     * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
//     * By setting a non-default limits, it effectively limits the output power of the close loop control.
//     *
//     * @param revLimit specifies the percentage output limit of the reverse direction.
//     * @param fwdLimit specifies the percentage output limit of the forward direction.
//     */
//    @Override
//    public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
//    {
//        // Allow this only if we are in close loop position control mode.
//        if (runMode == DcMotor.RunMode.RUN_TO_POSITION)
//        {
//            revLimit = Math.abs(revLimit);
//            fwdLimit = Math.abs(fwdLimit);
//
//            if (revLimit != fwdLimit)
//            {
//                throw new IllegalArgumentException("revLimit and fwdLimit must have the same magnitude.");
//            }
//
//            setNativeMotorPower(fwdLimit);
//        }
//    }   //setCloseLoopOutputLimits

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        throw new UnsupportedOperationException("Motor controller does not support setCloseLoopRampRate.");
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        throw new UnsupportedOperationException("Motor controller does not support setOpenLoopRampRate.");
    }   //setOpenLoopRampRate

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
        motor.setZeroPowerBehavior(enabled? DcMotor.ZeroPowerBehavior.BRAKE: DcMotor.ZeroPowerBehavior.FLOAT);
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //disableMotorFwdLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //isMotorFwdLimitSwitchEnabled

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorRevLimitSwitchInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //setMotorRevLimitSwitchInverted

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorFwdLimitSwitchInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //isMotorRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        throw new UnsupportedOperationException("Motor controller does not support limit switches.");
    }   //isMotorFwdLimitSwitchActive

    /**
     * This method sets the soft position limit for the reverse direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorRevSoftPositionLimit(Double limit)
    {
        throw new UnsupportedOperationException("Motor controller does not support soft limits.");
    }   //setMotorRevSoftPositionLimit

    /**
     * This method sets the soft position limit for the forward direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorFwdSoftPositionLimit(Double limit)
    {
        throw new UnsupportedOperationException("Motor controller does not support soft limits.");
    }   //setMotorFwdSoftPositionLimit

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setMotorPositionSensorInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Motor controller does not support position sensor.");
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        throw new UnsupportedOperationException("Motor controller does not support position sensor.");
    }   //isMotorPositionSensorInverted

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
                TrcTimer.getModeElapsedTime(), this, motor.getCurrentPosition());
        }

        DcMotor.RunMode prevMotorMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
                    TrcTimer.getModeElapsedTime(), this, motorPos);
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
                        funcName, "[%.3f] Reset %s success!", TrcTimer.getModeElapsedTime(), this);
                }
                break;
            }
        }

        if (motorPos != 0)
        {
            globalTracer.traceWarn(funcName, "motor %s encoder reset timed out (enc=%d).", this, motorPos);
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
        resetMotorPosition(DEF_POS_RESET_TIMEOUT);
    }   //resetMotorPosition

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        motor.setDirection(inverted? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.getDirection() == DcMotor.Direction.REVERSE;
    }   //isMotorInverted

    /**
     * This method sets the percentage motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        if (runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        {
            // Not in power control mode, set it so.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }
        motor.setPower(power);
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
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param vel specifies the motor velocity in raw sensor units per second (encoder counts per sec).
     */
    @Override
    public void setMotorVelocity(double vel)
    {
        if (runMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            // Not in velocity control mode, set it so.
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        }
        motor.setVelocity(vel);
        velocityTarget = vel;
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
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param pos specifies the motor position in raw sensor units (encoder counts).
     * @param powerLimit specifies the maximum power limit the motor will move.
     */
    @Override
    public void setMotorPosition(double pos, double powerLimit)
    {
        if (runMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runMode = DcMotor.RunMode.RUN_TO_POSITION;
        }
        motor.setPower(powerLimit);
        motor.setTargetPosition((int) pos);
        positionTarget = (int) pos;
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
        return motor.getCurrentPosition();
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        throw new UnsupportedOperationException("Motor controller does not support setMotorCurrent.");
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
     * This method sets the PID coefficients of the motor controller's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        motor.setVelocityPIDFCoefficients(pidCoeff.kP, pidCoeff.kI, pidCoeff.kD, pidCoeff.kF);
    }   //setMotorVelocityPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's velocity PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorVelocityPidTolerance(double tolerance)
    {
        velocityPidTolerance = tolerance;
    }   //setMotorVelocityPidTolerance

    /**
     * This method returns the PID coefficients of the motor controller's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorVelocityPidCoefficients()
    {
        TrcPidController.PidCoefficients pidCoeff;

        PIDFCoefficients pidfCoeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidCoeff = new TrcPidController.PidCoefficients(pidfCoeffs.p, pidfCoeffs.i, pidfCoeffs.d, pidfCoeffs.f);

        return pidCoeff;
    }   //getMotorVelocityPidCoefficients

    /**
     * This method checks if the motor is at the set velocity.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorVelocityOnTarget()
    {
        return Math.abs(velocityTarget - motor.getVelocity()) <= velocityPidTolerance;
    }   //getMotorVelocityOnTarget

    /**
     * This method sets the PID coefficients for the motor's position PID controller. For FTC motors, only kP makes
     * sense because of the double layer architecture (i.e. position PID is used in conjunction with velocity PID).
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        motor.setPositionPIDFCoefficients(pidCoeff.kP);
    }   //setMotorPositionPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's position PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorPositionPidTolerance(double tolerance)
    {
        motor.setTargetPositionTolerance((int) tolerance);
    }   //setMotorPositionPidTolerance

    /**
     * This method returns the PID coefficients of the motor controller's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorPositionPidCoefficients()
    {
        TrcPidController.PidCoefficients pidCoeff;

        PIDFCoefficients pidfCoeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        pidCoeff = new TrcPidController.PidCoefficients(pidfCoeffs.p, pidfCoeffs.i, pidfCoeffs.d, pidfCoeffs.f);

        return pidCoeff;
    }   //getMotorPositionPidCoefficients

    /**
     * This method checks if the motor is at the set position.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorPositionOnTarget()
    {
        return Math.abs(positionTarget - motor.getCurrentPosition()) <= motor.getTargetPositionTolerance();
    }   //getMotorPositionOnTarget

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("Motor controller does not support setMotorCurretPidCoefficients.");
    }   //setMotorCurrentPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's current PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorCurrentPidTolerance(double tolerance)
    {
        throw new UnsupportedOperationException("Motor controller does not support setMotorCurretPidTolerance.");
    }   //setMotorCurrentPidTolerance

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients()
    {
        throw new UnsupportedOperationException("Motor controller does not support getMotorCurretPidCoefficients.");
    }   //geteMotorCurrentPidCoefficients

    /**
     * This method checks if the motor is at the set current.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorCurrentOnTarget()
    {
        throw new UnsupportedOperationException("Motor controller does not support getMotorCurretOnTarget.");
    }   //getMotorCurrentOnTarget

    //
    // The following methods override the software simulation in TrcMotor providing direct support in hardware.
    //

}   //class FtcDcMotor
