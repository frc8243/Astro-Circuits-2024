
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



    public class LEDs extends SubsystemBase {
        private final static AddressableLED LED = new AddressableLED(0);
        private final static AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(60);
     
         /** Creates a new LEDs. */
  public LEDs() {
    LED.setLength(LEDBuffer.getLength());
    LED.setData(LEDBuffer);
    LED.start();
    }
    
    @Override
    public void periodic() {
      
      LED.setData(LEDBuffer);
  
    }



    public static void frontHalf(int r, int g, int b) {
        for (int i = 0; i < LEDBuffer.getLength()/2; i++) {
          LEDBuffer.setRGB(i, r, g, b);
        }
      } 



      public static void backHalf(int r, int g, int b) {
        for (int i = LEDBuffer.getLength()/2+1; i < 60;i++) {
          LEDBuffer.setRGB(i, r, g, b);
        } 
      }



      public static void allLEDS(int r, int g, int b) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
          LEDBuffer.setRGB(i, r, g, b);  
        }
      }   
     
      
      
    }


