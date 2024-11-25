The LED Display system in this library is meant to be used with the sprite system contained in the FTC-core library. The custom designed Adafruit driver contained within this FTC-base library is meant to be run on the Adafruit Neopixel driver, which supports a maximum of 512 LEDs. The Visual Display subsystem works with this, taking in inputs from the sprite system on which LEDs to light up and outputting this info to the Adafruit driver, which then writes the signals to the physical LEDs on the robot. The only thing that needs to be done in order to use this system is modify the findLedArrayIndex method in the VisualDisplaySubsystem class to match the layout of the LEDs on your robot. This method is used to convert the x and y coordinates of the sprite system to the index of the LED so that the Adafruit driver can know which LED to light up.

## How to modify findLedArrayIndex method
In order to modify this method, you have to figure out the layout of the LEDs on your robot. Most conventional LEDs are either wired in a snake pattern or a grid pattern. If using a snake pattern, you can use the following code:
```java
if (ledX % 2 == 0) {
            return ledX * HEIGHT_OF_LED_ARRAY + ledY;
        } else {
            return ledX * HEIGHT_OF_LED_ARRAY + (HEIGHT_OF_LED_ARRAY - 1) - ledY;
        }
```

If using a grid pattern, you can use the following code:
```java
return ledX * HEIGHT_OF_LED_ARRAY + ledY;
```

