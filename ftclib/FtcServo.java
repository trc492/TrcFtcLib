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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements a platform dependent servo extending TrcServo. It provides implementation of the abstract
 * methods in TrcServo.
 */
public class FtcServo extends TrcServo
{
    private static final String moduleName = "FtcServo";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private enum State
    {
        SET_POSITION,
        DISABLE_CONTROLLER,
        DONE
    }   //enum State

    private static final double CONTROLLER_ONOFF_DELAY = 0.1;

    private final Servo servo;
    private final ServoController controller;
    private final TrcTimer holdTimer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject servoTaskObj;
    private double servoPos = 0.0;
    private double servoOnTime = 0.0;
    private Double logicalPos;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param continuous specifies true if it is a continuous servo, false otherwise.
     */
    public FtcServo(HardwareMap hardwareMap, String instanceName, boolean continuous)
    {
        super(instanceName, continuous);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        servo = hardwareMap.get(Servo.class, instanceName);
        logicalPos = null;
        controller = servo.getController();
        holdTimer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
        sm = new TrcStateMachine<>(instanceName);
        servoTaskObj = TrcTaskMgr.createTask(instanceName + ".servoTask", this::servoTask);
    }   //FtcServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param continuous specifies true if it is a continuous servo, false otherwise.
     */
    public FtcServo(String instanceName, boolean continuous)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, continuous);
    }   //FtcServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcServo(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, false);
    }   //FtcServo

    /**
     * This method returns the servo controller object that this servo is plugged into.
     *
     * @return servo controller object.
     */
    public ServoController getController()
    {
        final String funcName = "getController";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, controller.toString());
        }

        return controller;
    }   //getController

    /**
     * This method cancels the holdTimer and stops the state machine if it is running.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (sm.isEnabled())
        {
            holdTimer.cancel();
            sm.stop();
            setTaskEnabled(false);
        }
    }   //cancel

    /**
     * This method enables/disables the periodic task that runs the state machine.
     *
     * @param enabled specifies true to enable the state machine task, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            servoTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        }
        else
        {
            servoTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method sets the servo position but will cut power to the servo when done. Since servo motors can't really
     * take a lot of loads, it would stress out and may burn out the servo if it is held against a heavy load for
     * extended period of time. This method allows us to set the position and only hold it long enough for it to
     * reach target position and then we will cut the servo controller power off. Note that by doing so, all servos
     * on the same controller will go limp.
     *
     * @param pos specifies the target position.
     * @param onTime specifies the time in seconds to wait before disabling servo controller.
     */
    public void setPositionWithOnTime(double pos, double onTime)
    {
        cancel();
        servoPos = pos;
        servoOnTime = onTime;
        sm.start(State.SET_POSITION);
        setTaskEnabled(true);
    }   //setPositionWithOnTime

    /**
     * The method eanbles/disables the servo controller. If the servo controller is disabled, all servos on the
     * controller will go limp. This is useful for preventing the servos from burning up if it is held against
     * a heavy load.
     *
     * @param on specifies true to enable the servo controller, false otherwise.
     */
    public void setControllerOn(boolean on)
    {
        final String funcName = "setControllerOn";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "on=%s", Boolean.toString(on));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (on)
        {
            controller.pwmEnable();
        }
        else
        {
            controller.pwmDisable();
        }
    }   //setControllerOn

    //
    // Implements TrcServo abstract methods.
    //

    /**
     * This methods inverts the servo motor direction.
     *
     * @param inverted specifies true if the servo direction is inverted, false otherwise.
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

        servo.setDirection(inverted? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
    }   //setInverted

    /**
     * This method returns true if the servo direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        final String funcName = "isInverted";
        boolean inverted = servo.getDirection() == Servo.Direction.REVERSE;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
        }

        return inverted;
    }   //isInverted

    /**
     * This method sets the logical position of the servo motor.
     *
     * @param position specifies the logical position of the servo motor in the range of [0.0, 1.0].
     */
    @Override
    public void setLogicalPosition(double position)
    {
        final String funcName = "setLogicalPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (logicalPos == null || position != logicalPos)
        {
            if (servoSetPosElapsedTimer != null) servoSetPosElapsedTimer.recordStartTime();
            servo.setPosition(position);
            if (servoSetPosElapsedTimer != null) servoSetPosElapsedTimer.recordEndTime();
            logicalPos = position;
        }
    }   //setLogicalPosition

    /**
     * This method returns the logical position value set by the last setLogicalPosition call. Note that servo motors
     * do not provide real time position feedback. Therefore, getLogicalPosition doesn't actually return the current
     * position.
     *
     * @return motor position value set by the last setLogicalPosition call in the range of [0.0, 1.0].
     */
    @Override
    public double getLogicalPosition()
    {
        final String funcName = "getLogicalPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", logicalPos);
        }

        return logicalPos != null? logicalPos: 0.0;
    }   //getLogicalPosition

    /**
     * This method stops a continuous servo. It doesn't do anything if the servo is not continuous.
     */
    public void stopContinuous()
    {
        if (isContinuous())
        {
            setLogicalPosition(SERVO_CONTINUOUS_STOP);
        }
    }   //stopContinuous

    /**
     * This method is called periodically to run a state machine that will enable the servo controller, set the servo
     * position, wait for the specified hold time, and finally disable the servo controller.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    private void servoTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case SET_POSITION:
                    setPosition(servoPos);
                    holdTimer.set(servoOnTime, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DISABLE_CONTROLLER);
                    break;

                case DISABLE_CONTROLLER:
                    controller.pwmDisable();
                    holdTimer.set(CONTROLLER_ONOFF_DELAY, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    sm.stop();
                    setTaskEnabled(false);
            }
        }
    }   //servoTask

}   //class FtcServo
