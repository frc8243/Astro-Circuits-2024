package frc.utils;

import java.lang.Math;
//8243 original code but man...i'm hungry for some sushi...

public class Normalization {
  public static double cube(double joystickVal) {
    return Math.pow(joystickVal, 3);
  }

  public static double square(double joystickVal) {
    return Math.pow(joystickVal, 2);
  }

  public static double linearDeadzone(double joystickVal, double deadzone) {
    return ((Math.abs(joystickVal) > deadzone) ? joystickVal : 0);
  }

  public static double logistic(double joystickVal, double k) {
    return 2 / (1 + (Math.pow((Math.E), -k * (joystickVal)))) - 1;
  }
}