package frc.robot.util;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class RevColorSensor {
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public Color getColor() {
        return colorSensor.getColor();
    }

   /**
   * Get the raw proximity value from the sensor ADC (11 bit). This value is largest when an object
   * is close to the sensor and smallest when far away.
   *
   * @return Proximity measurement value, ranging from 0 to 2047
   */
    public int getProximity() {
        return colorSensor.getProximity();
    }
}
