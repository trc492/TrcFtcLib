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

import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcWatchdogMgr;

/**
 * This class implements a cooperative multi-tasking scheduler extending LinearOpMode.
 */
public abstract class FtcOpMode extends LinearOpMode implements TrcRobot.RobotMode
{
    private static final String moduleName = "FtcOpMode";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static String opModeName = null;
    private Thread robotThread;
    private TrcWatchdogMgr.Watchdog robotThreadWatchdog;
    private TextToSpeech textToSpeech = null;

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    public abstract void initRobot();

    protected final static int NUM_DASHBOARD_LINES = 16;
    private final static long LOOP_PERIOD_NANO = 50000000;
    private static FtcOpMode instance = null;
    private static long loopStartNanoTime = 0;
    private static long loopCounter = 0;

    private long slowPeriodicTotalElapsedTime = 0;
    private int slowPeriodicTimeSlotCount = 0;
    private long fastPeriodicTotalElapsedTime = 0;
    private int fastPeriodicTimeSlotCount = 0;
    private long sdkTotalElapsedTime = 0;
    private final Object startNotifier;

    /**
     * Constructor: Creates an instance of the object. It calls the constructor of the LinearOpMode class and saves
     * an instance of this class.
     */
    public FtcOpMode()
    {
        super();

        //
        // We must set DbgLog as early as possible before any instantiation of TrcDbgTrace because TrcDbgTrace must
        // have it or it will throw an exception.
        //
        TrcDbgTrace.setDbgLog(new FtcDbgLog());
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
        try
        {
            Field runningNotifierField = LinearOpMode.class.getDeclaredField("runningNotifier");
            runningNotifierField.setAccessible(true);
            startNotifier = runningNotifierField.get(this);
        }
        catch (NoSuchFieldException | IllegalAccessException e)
        {
            e.printStackTrace();
            throw new RuntimeException("Failed to access runningNotifier.");
        }
    }   //FtcOpMode

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FtcOpMode getInstance()
    {
        if (instance == null) throw new NullPointerException("You are not using FtcOpMode!");
        return instance;
    }   //getInstance

    /**
     * This method returns the name of the active OpMode.
     *
     * @return active OpMode name.
     */
    public static String getOpModeName()
    {
        return opModeName;
    }   //getOpModeName

    /**
     * This method returns the start time of the time slice loop. This is useful for the caller to determine if it
     * is in the same time slice as a previous operation for optimization purposes.
     *
     * @return time slice loop start time.
     */
    public static double getLoopStartTime()
    {
        return loopStartNanoTime/1000000000.0;
    }   //getElapsedTime

    /**
     * This method returns the loop counter. This is very useful for code to determine if it is called multiple times
     * in the same loop. For example, it can be used to optimize sensor access so that if the sensor is accessed in
     * the same loop, there is no reason to create a new bus transaction to get "fresh" data from the sensor.
     *
     * @return loop counter value.
     */
    public static long getLoopCounter()
    {
        return loopCounter;
    }   //getLoopCounter

    /**
     * This method returns a TextToSpeech object. If it doesn't exist yet, one is created.
     *
     * @param locale specifies the language locale.
     * @return TextToSpeech object.
     */
    public TextToSpeech getTextToSpeech(final Locale locale)
    {
        if (textToSpeech == null)
        {
            textToSpeech = new TextToSpeech(
                    hardwareMap.appContext,
                    status -> {
                        if (status != TextToSpeech.ERROR)
                        {
                            textToSpeech.setLanguage(locale);
                        }
                    });
        }

        return textToSpeech;
    }   //getTextToSpeech

    /**
     * This method returns a TextToSpeech object with US locale.
     *
     * @return TextToSpeech object.
     */
    public TextToSpeech getTextToSpeech()
    {
        return getTextToSpeech(Locale.US);
    }   //getTextToSpeech

    /**
     * This method returns the annotation object of the specifies opmode type if it is present.
     *
     * @param opmodeType specifies the opmode type.
     * @return annotation object of the specified opmode type if present, null if not.
     */
    public Annotation getOpmodeAnnotation(Class opmodeType)
    {
        return getClass().getAnnotation(opmodeType);
    }   //getOpmodeAnnotation

    /**
     * This method returns the opmode type name.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type name.
     */
    public String getOpmodeTypeName(Class<?> opmodeType)
    {
        String opmodeTypeName = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeName = ((Autonomous)annotation).name();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeName = ((TeleOp)annotation).name();
            }
        }

        return opmodeTypeName;
    }   //getOpmodeTypeName

    /**
     * This method returns the opmode type group.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type group.
     */
    public String getOpmodeTypeGroup(Class<?> opmodeType)
    {
        String opmodeTypeGroup = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeGroup = ((Autonomous)annotation).group();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeGroup = ((TeleOp)annotation).group();
            }
        }

        return opmodeTypeGroup;
    }   //getOpmodeTypeGroup

    /**
     * This method enables/disables Bulk Caching Mode. It is useful in situations such as resetting encoders and
     * reading encoder values in a loop waiting for it to be cleared. With caching mode ON, the encoder value may
     * never get cleared.
     *
     * @param enabled specifies true to enable caching mode, false to disable.
     */
    public void setBulkCachingModeEnabled(boolean enabled)
    {
        LynxModule.BulkCachingMode cachingMode = enabled? LynxModule.BulkCachingMode.MANUAL: LynxModule.BulkCachingMode.OFF;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module: allHubs)
        {
            module.setBulkCachingMode(cachingMode);
        }
    }   //setBulkCachingModeEnabled

    /**
     * This method clears the bulk cache if the module is in Manual mode.
     */
    public void clearBulkCacheInManualMode()
    {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module: allHubs)
        {
            if (module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL)
            {
                module.clearBulkCache();
            }
        }
    }   //clearBulkCacheInManualMode

    /**
     * This method sends a heart beat to the main robot thread watchdog. This is important if during robot init time
     * that the user code decided to synchronously busy wait for something, it must periodically call this method to
     * prevent the watchdog from complaining.
     */
    public void sendWatchdogHeartBeat()
    {
        final String funcName = "sendWatchdogHeartBeat";

        if (Thread.currentThread() == robotThread)
        {
            if (robotThreadWatchdog != null)
            {
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            else
            {
                TrcDbgTrace.globalTraceWarn(funcName, "Robot thread watchdog has not been created yet.");
                TrcDbgTrace.printThreadStack();
            }
        }
        else
        {
            TrcDbgTrace.globalTraceWarn(funcName, "Caller must be on the OpMode thread to call this.");
            TrcDbgTrace.printThreadStack();
        }
    }   //sendWatchdogHeartBeat

    //
    // Implements LinearOpMode
    //

    /**
     * This method is called when our OpMode is loaded and the "Init" button on the Driver Station is pressed.
     */
    @Override
    public void runOpMode()
    {
        final String funcName = "runOpMode";
        //
        // Create dashboard here. If any earlier, telemetry may not exist yet.
        //
        FtcDashboard dashboard = FtcDashboard.createInstance(telemetry, NUM_DASHBOARD_LINES);
        TrcRobot.RunMode runMode;

        if (debugEnabled)
        {
            if (dbgTrace == null)
            {
                dbgTrace = new TrcDbgTrace(
                        moduleName, false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
            }
        }
        //
        // Determine run mode. Note that it means the OpMode must be annotated with group="FtcAuto", group="FtcTeleOp"
        // or group="FtcTest".
        //
        opModeName = getOpmodeTypeName(Autonomous.class);
        if (opModeName != null)
        {
            runMode = TrcRobot.RunMode.AUTO_MODE;
        }
        else
        {
            opModeName = getOpmodeTypeName(TeleOp.class);
            if (opModeName != null)
            {
                if (getOpmodeTypeGroup(TeleOp.class).startsWith("FtcTest") ||
                    getOpmodeTypeName(TeleOp.class).startsWith("FtcTest"))
                {
                    runMode = TrcRobot.RunMode.TEST_MODE;
                }
                else
                {
                    runMode = TrcRobot.RunMode.TELEOP_MODE;
                }
            }
            else
            {
                throw new IllegalStateException(
                        "Invalid OpMode annotation, OpMode must be annotated with either @Autonomous or @TeleOp.");
            }
        }
        TrcRobot.setRunMode(runMode);

        robotThread = Thread.currentThread();
        robotThreadWatchdog = TrcWatchdogMgr.registerWatchdog(Thread.currentThread().getName() + ".watchdog");
        TrcEvent.registerEventCallback();

        if (TrcMotor.getNumOdometryMotors() > 0)
        {
            if (debugEnabled)
            {
                dbgTrace.traceWarn(funcName, "Odometry motors list is not empty (numMotors=%d)!",
                        TrcMotor.getNumOdometryMotors());
            }
            TrcMotor.clearOdometryMotorsList(true);
        }

        setBulkCachingModeEnabled(true);
        //
        // Initialize mode start time before match starts in case somebody calls TrcUtil.getModeElapsedTime before
        // competition starts (e.g. in initRobot) so it will report elapsed time from the "Init" button being pressed.
        //
        TrcUtil.recordModeStartTime();

        try
        {
            //
            // robotInit contains code to initialize the robot.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Current RunMode: %s", runMode);
                dbgTrace.traceInfo(funcName, "Running initRobot");
            }
            dashboard.displayPrintf(0, "initRobot starting...");
            // Note: initRobot is synchronous, nothing periodic will be processed until it comes back.
            initRobot();
            dashboard.displayPrintf(0, "initRobot completed!");

            //
            // Run initPeriodic while waiting for competition to start.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running initPeriodic");
            }
            loopCounter = 0;
            dashboard.displayPrintf(0, "initPeriodic starting...");
            while (!isStarted())
            {
                loopCounter++;
                loopStartNanoTime = TrcUtil.getNanoTime();

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%d:%.3f]: InitPeriodic loop",
                            loopCounter, loopStartNanoTime/1000000000.0);
                }

                clearBulkCacheInManualMode();
                initPeriodic();
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            dashboard.displayPrintf(0, "initPeriodic completed!");
            TrcUtil.recordModeStartTime();

            //
            // Prepare for starting the run mode.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running Start Mode Tasks");
            }
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, runMode);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running startMode");
            }
            startMode(null, runMode);

            long nextPeriodNanoTime = TrcUtil.getNanoTime();
            long startNanoTime = TrcUtil.getNanoTime();

            loopCounter = 0;
            while (opModeIsActive())
            {
                loopStartNanoTime = TrcUtil.getNanoTime();
                loopCounter++;
                sdkTotalElapsedTime += loopStartNanoTime - startNanoTime;
                double opModeElapsedTime = TrcUtil.getModeElapsedTime();
                boolean slowLoop = loopStartNanoTime >= nextPeriodNanoTime;

                clearBulkCacheInManualMode();
                //
                // Fast Pre-Periodic Task
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                        funcName, "[%d:%.3f]: OpMode loop", loopCounter, loopStartNanoTime/1000000000.0);
                    dbgTrace.traceInfo(funcName, "Running FastPrePeriodic Tasks");
                }
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.FAST_PREPERIODIC_TASK, runMode);
                //
                // Slow Pre-Periodic Task
                //
                if (slowLoop)
                {
                    nextPeriodNanoTime += LOOP_PERIOD_NANO;
                    dashboard.displayPrintf(0, "%s: %.3f", opModeName, opModeElapsedTime);

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running SlowPrePeriodic Tasks");
                    }
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.SLOW_PREPERIODIC_TASK, runMode);
                }
                //
                // Fast Periodic
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running fastPeriodic");
                }
                startNanoTime = TrcUtil.getNanoTime();
                fastPeriodic(opModeElapsedTime);
                fastPeriodicTotalElapsedTime += TrcUtil.getNanoTime() - startNanoTime;
                fastPeriodicTimeSlotCount++;
                //
                // Slow Periodic
                //
                if (slowLoop)
                {
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running slowPeriodic");
                    }
                    startNanoTime = TrcUtil.getNanoTime();
                    slowPeriodic(opModeElapsedTime);
                    slowPeriodicTotalElapsedTime += TrcUtil.getNanoTime() - startNanoTime;
                    slowPeriodicTimeSlotCount++;
                }
                //
                // Fast Post-Periodic Task
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running FastPostPeriodic Tasks");
                }
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK, runMode);
                //
                // Slow Post-Periodic Task
                //
                if (slowLoop)
                {
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running SlowPostPeriodic Tasks");
                    }

                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK, runMode);
                }

                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
                //
                // Letting FTC SDK do its things.
                //
                startNanoTime = TrcUtil.getNanoTime();
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running stopMode");
            }
            stopMode(runMode, null);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running Stop Mode Tasks");
            }
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, runMode);
        }
        finally
        {
            //
            // Make sure we properly clean up and shut down even if the code throws an exception but we are not
            // catching the exception and let it propagate up.
            //
            TrcEvent.unregisterEventCallback();
            if (robotThreadWatchdog != null)
            {
                robotThreadWatchdog.unregister();
            }
            robotThreadWatchdog = null;
            TrcMotor.clearOdometryMotorsList(true);
            TrcTaskMgr.shutdown();
        }
    }   //runOpMode

    /**
     * This method prints the performance metrics of all loops and taska.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        tracer.traceInfo(
                moduleName,
                "%16s: SlowPeriodic=%.6f, FastPeriodic=%.6f, SDK=%.6f",
                opModeName,
                (double)slowPeriodicTotalElapsedTime/slowPeriodicTimeSlotCount/1000000000,
                (double)fastPeriodicTotalElapsedTime/fastPeriodicTimeSlotCount/1000000000,
                (double)sdkTotalElapsedTime/loopCounter/1000000000);
        TrcTaskMgr.printTaskPerformanceMetrics(tracer);
    }   //printPerformanceMetrics

    /**
     * This method is called periodically after initRobot() is called but before competition starts. Typically,
     * you override this method and put code that will check and display robot status in this method. For example,
     * one may monitor the gyro heading in this method to make sure there is no major gyro drift before competition
     * starts. By default, this method is doing exactly what waitForStart() does.
     */
    public void initPeriodic()
    {
        synchronized (startNotifier)
        {
            try
            {
                startNotifier.wait();
            }
            catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }
    }   //initPeriodic

    //
    // Implements TrcRobot.RobotMode interface.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
     * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
     * sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //stopMode

    /**
     * This method is called periodically at a fast rate. Typically, you put code that requires servicing at a
     * high frequency here. To make the robot as responsive and as accurate as possible especially in autonomous
     * mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void fastPeriodic(double elapsedTime)
    {

    }   //fastPeriodic

    /**
     * This method is called periodically at a slow rate. Typically, you put code that doesn't require frequent
     * update here. For example, TeleOp joystick code or status display code can be put here since human responses
     * are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void slowPeriodic(double elapsedTime)
    {
    }   //slowPeriodic

}   //class FtcOpMode
