/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.archive;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.archive.TrcEncoder;
import trclib.archive.TrcWrapValueConverter;

/**
 * This class implements a wrapper to OctoQuad that supports up to 8 quadrature or PWM encoders. It implements the
 * TrcEncoder interface to allow compatibility to other types of encoders.
 */
public class FtcOctoQuad implements TrcEncoder
{
    private static OctoQuad octoQuad = null;
    private final int encIndex;
    private final TrcWrapValueConverter wrapValueConverter;
    private double sign = 1.0;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param encIndex specifies the encoder port index.
     * @param wrapValueLow specifies the encoder wrap range low value, null if value is continuous.
     * @param wrapValueHigh specifies the encoder wrap range high value, null if value is continuous.
     */
    public FtcOctoQuad(
        HardwareMap hardwareMap, String instanceName, int encIndex, Double wrapValueLow, Double wrapValueHigh)
    {
        if (octoQuad == null)
        {
            octoQuad = hardwareMap.get(OctoQuad.class, instanceName);
        }

        this.encIndex = encIndex;
        if (wrapValueLow != null && wrapValueHigh != null)
        {
            wrapValueConverter = new TrcWrapValueConverter(
                instanceName, this::getRawPosition, wrapValueLow, wrapValueHigh);
        }
        else
        {
            wrapValueConverter = null;
        }
    }   //FtcOctoQuad

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param encIndex specifies the encoder port index.
     * @param wrapValueLow specifies the encoder wrap range low value, null if value is continuous.
     * @param wrapValueHigh specifies the encoder wrap range high value, null if value is continuous.
     */
    public FtcOctoQuad(String instanceName, int encIndex, Double wrapValueLow, Double wrapValueHigh)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, encIndex, wrapValueLow, wrapValueHigh);
    }   //FtcOctoQuad

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param encIndex specifies the encoder port index.
     */
    public FtcOctoQuad(String instanceName, int encIndex)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, encIndex, null, null);
    }   //FtcOctoQuad

    /**
     * This method enables/disables the Wrap Value Converter task.
     *
     * @param enabled specifies true to enable wrap value converter, false to disable.
     */
    public void setWrapConverterEnabled(boolean enabled)
    {
        if (wrapValueConverter != null)
        {
            wrapValueConverter.setTaskEnabled(enabled);
        }
    }   //setWrapConverterEnabled

    /**
     * This method checks if the Wrap Value Converter task is enabled.
     *
     * @return true if wrap value converter is enabled, false if disabled.
     */
    public boolean isWrapConverterEnabled()
    {
        return wrapValueConverter != null && wrapValueConverter.isTaskEnabled();
    }   //isWrapConverterEnabled

    //
    // Implements the TrcEncoder interface.
    //

    /**
     * This method resets the position of the specified encoder.
     */
    @Override
    public void reset()
    {
        octoQuad.resetSinglePosition(encIndex);
        if (wrapValueConverter != null)
        {
            wrapValueConverter.resetConverter();
        }
    }   //reset

    /**
     * This method reads the raw position of the specified encoder.
     *
     * @return raw encoder position.
     */
    @Override
    public double getRawPosition()
    {
        return octoQuad.readSinglePosition(encIndex);
    }   //getRawPosition

    /**
     * This method returns the position of the specified encoder adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        double value = wrapValueConverter != null? wrapValueConverter.getContinuousValue(): getRawPosition();
        return (value - zeroOffset) * scale * sign + offset;
    }   //getScaledPosition

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        this.sign = inverted? -1.0: 1.0;
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is reversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return sign < 0.0;
    }   //isInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset, double zeroOffset)
    {
        this.scale = scale;
        this.offset = offset;
        this.zeroOffset = zeroOffset;
    }   //setScaleAndOffset

}   //class FtcOctoQuad
