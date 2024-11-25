package team.techtigers.base.visualdisplay;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import team.techtigers.core.display.Color;

/**
 * Driver for the Adafruit NeoPixel driver. This driver can be used to control individually
 * addressable LEDs over I2C.
 * <p>
 * See <a href="https://learn.adafruit.com/adafruit-neodriver-i2c-to-neopixel-driver"> Product website</a> for more information.
 */
@I2cDeviceType
@DeviceProperties(name = "Adafruit NeoPixel Driver", description = "a NeoPixel driver over I2C", xmlTag = "AdafruitNeoPixel")
public class AdafruitNeoPixel extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x60);
    private static final byte REGISTER_BASE = 0x0E;
    private static final byte REGISTER_PIN = 0x01;
    private static final byte REGISTER_SPEED = 0x02;
    private static final byte REGISTER_BUF_LENGTH = 0x03;
    private static final byte REGISTER_BUF = 0x04;
    private static final byte REGISTER_SHOW = 0x05;
    /* Device will only allow 512 LEDs to be written to,
     see: https://learn.adafruit.com/adafruit-neodriver-i2c-to-neopixel-driver */
    private static final int MAX_LEDS = 512;
    private static final int MAX_SEQUENCE_LENGTH = 30;

    private int bytesPerLed;
    protected Color[] currentLedBuffer;
    protected Color[] lastLedBuffer;
    protected Color[] emptyLedBuffer;

    /**
     * Creates a new Adafruit NeoPixel driver.
     *
     * @param i2cDeviceSynch      this item is traditioanally created through the hardwareMap
     * @param deviceClientIsOwned this item is traditioanally created through the hardwareMap
     */
    public AdafruitNeoPixel(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();

        bytesPerLed = 3;
        currentLedBuffer = new Color[]{};
        lastLedBuffer = new Color[]{};
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    private void writeDevicePin(byte pin) {
        deviceClient.write(new byte[]{
                REGISTER_BASE,
                REGISTER_PIN,
                pin
        });
    }

    private void writeDeviceSpeed(byte speed) {
        deviceClient.write(new byte[]{
                REGISTER_BASE,
                REGISTER_SPEED,
                speed
        });
    }

    private void writeDeviceBufferLength(short length) {
        deviceClient.write(new byte[]{
                REGISTER_BASE,
                REGISTER_BUF_LENGTH,
                (byte) (length >> 8),
                (byte) (length & 0xFF)
        });
    }

    private void writeDeviceBuffer(byte[] payload) {
        if (payload.length > MAX_SEQUENCE_LENGTH + 2) {
            throw new IllegalArgumentException("Payload cannot be longer than " + (MAX_SEQUENCE_LENGTH + 2) + " bytes");
        }
        byte[] buffer = new byte[payload.length + 2];
        buffer[0] = REGISTER_BASE;
        buffer[1] = REGISTER_BUF;
        System.arraycopy(payload, 0, buffer, 2, payload.length);
        deviceClient.write(buffer);
    }

    private void writeDeviceShow() {
        deviceClient.write(new byte[]{REGISTER_BASE,
                REGISTER_SHOW
        });
    }

    private byte[] createPayload(int startIndex, int length) {
        byte[] payload = new byte[length * bytesPerLed + 2];
        int startAddress = startIndex * bytesPerLed;

        payload[0] = (byte) (startAddress >> 8);
        payload[1] = (byte) (startAddress & 0xFF);

        for (int index = 0; index < length; index++) {
            Color ledColor = currentLedBuffer[startIndex + index];
            int payloadIndex = (index * bytesPerLed) + 2;

            payload[payloadIndex] = ledColor.green;
            payload[payloadIndex + 1] = ledColor.red;
            payload[payloadIndex + 2] = ledColor.blue;
            if (bytesPerLed == 4) {
                payload[payloadIndex + 3] = ledColor.white;
            }
        }

        return payload;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit NeoPixel Driver";
    }

    /**
     * Initializes the driver to have an amount of LEDs with each LED having a certain amount of
     * bytes.
     *
     * @param numberOfLeds The amount of LEDs that can be written to
     * @param bytesPerLed  The amount of bytes that should be written to each LED.
     *                     3 bytes for RGB, 4 bytes for RGBW.
     */
    public void initialize(int numberOfLeds, int bytesPerLed) {
        if (numberOfLeds > MAX_LEDS) {
            throw new IllegalArgumentException("Cannot have more than " + MAX_LEDS + " LEDs");
        } else if (numberOfLeds < 0) {
            throw new IllegalArgumentException("Cannot have less than 0 LEDs");
        }

        if (bytesPerLed != 3 && bytesPerLed != 4) {
            throw new IllegalArgumentException("Bytes per LED must be 3 or 4");
        }

        // Set the number of bytes per LED
        this.bytesPerLed = bytesPerLed;

        // Creating buffers
        currentLedBuffer = new Color[numberOfLeds];
        emptyLedBuffer = new Color[numberOfLeds];
        lastLedBuffer = new Color[numberOfLeds];

        for (int index = 0; index < numberOfLeds; index++) {
            currentLedBuffer[index] = Color.BLACK;
            emptyLedBuffer[index] = Color.BLACK;
            lastLedBuffer[index] = Color.RED;
        }

        // Set pin to 15, according to the docs:
        writeDevicePin((byte) 0x0F);

        // Set NeoPixel protocol frequency to 800khz
        writeDeviceSpeed((byte) 0x01);

        // Set the buffer length to the number of LEDs * bytes per LED
        writeDeviceBufferLength((short) (numberOfLeds * bytesPerLed));
    }

    /**
     * Sets the colors for a set of LEDs in a buffer. This will set it starting from the id given
     * and set every following LED to the next color until it runs out of colors to set.
     *
     * @param index  The index of the first LED
     * @param colors The colors to set the LEDs to
     */
    public void setLeds(int index, Color[] colors) {
        if (index < 0 || index >= currentLedBuffer.length) {
            throw new IllegalArgumentException("Index must be between 0 and " + currentLedBuffer.length);
        }
        if (colors == null || colors.length == 0) {
            throw new IllegalArgumentException("Colors cannot be null or empty");
        }
        if (colors.length + index > currentLedBuffer.length) {
            throw new IllegalArgumentException("Color values will overflow LED buffer");
        }

        for (int colorIndex = 0; colorIndex < colors.length; colorIndex++) {
            Color inputColor = colors[colorIndex];
            Color ledColor = currentLedBuffer[index + colorIndex];

            if (inputColor == null) {
                inputColor = Color.BLACK;
                colors[colorIndex] = inputColor;
            }

            if (!inputColor.equals(ledColor)) {
                currentLedBuffer[index + colorIndex] = colors[colorIndex];
            }
        }
    }

    /**
     * Overloaded method to set only one led to a certain color in a buffer.
     *
     * @param index The index of the LED
     * @param color The color to set the LED to
     */
    public void setLeds(int index, Color color) {
        setLeds(index, new Color[]{color});
    }

    /**
     * Clears all the LEDs in the buffer, making them all be off when shown.
     */
    public void clearLeds() {
        setLeds(0, emptyLedBuffer);
    }

    /**
     * Shows the LEDs in the buffer on the actual LEDs. This will only do this with LEDs that have
     * changed since the last update.
     */
    public void show() {
        int startIndex = -1;
        int length = 0;
        boolean showLeds = false;
        for (int index = 0; index < currentLedBuffer.length; index++) {
            boolean hasChanged = !currentLedBuffer[index].equals(lastLedBuffer[index]);
            if (hasChanged) {
                if (startIndex == -1) {
                    startIndex = index;
                }
                length++;
                lastLedBuffer[index] = currentLedBuffer[index];
                showLeds = true;
            }

            // 1. Buffer has not changed and tracking a sequence
            boolean endOfSequence = !hasChanged && startIndex > -1;
            // 2. Length of sequence is > max sequence length
            boolean maxLengthReached = length >= MAX_SEQUENCE_LENGTH / bytesPerLed;
            // 3. End of the buffer and tracking a sequence
            boolean trackingEndOfBuffer = startIndex > -1 && index == lastLedBuffer.length - 1;

            if (endOfSequence || maxLengthReached || trackingEndOfBuffer) {
                // Write the sequence to the device
                writeDeviceBuffer(createPayload(startIndex, length));
                startIndex = -1;
                length = 0;
            }
        }

        if (showLeds) {
            writeDeviceShow();
        }
    }
}
