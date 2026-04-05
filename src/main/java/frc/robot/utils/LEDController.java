package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDController {

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    public LEDController(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setAll(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds.setData(buffer);
    }
    public void setOne(double percentage, int r, int g, int b) {
        int i = (int)Math.round(percentage*buffer.getLength());
        buffer.setRGB(i,r,g,b);
        leds.setData(buffer);
    }
    public void setMultiple(double percentage, int r, int g, int b) {
        int max = (int)Math.round(percentage*buffer.getLength());
        for (int i = 0; i < max; i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds.setData(buffer);
    }
    public void setMultiple2(double minPercentage, double maxPercentage,int r, int g, int b) {
        int max = (int)Math.round(maxPercentage*buffer.getLength());
        int min = (int)Math.round(minPercentage*buffer.getLength());
        for (int i = min; i < max; i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds.setData(buffer);
    }
    public void zeroAll() {
               for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0,0);
        }
        leds.setData(buffer); 
    }
}