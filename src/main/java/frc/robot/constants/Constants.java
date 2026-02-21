package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class LauncherConstants {
    public static final int UpperMotorPort = 14; 
    public static final int LowerMotorPort = 13; 
    public static final int ConveyerMotorPort = 15; 

     public static final double UpperMotorSpeedRpm = 2062;
        public static final double LowerMotorSpeedRpm  = 1030;
        public static final double ConveyerMotorSpeedRpm = 0.3;
    //lower motor double the speed top
}
public static class IndexerConstants {
  public static final int kIndexMotorPort = 16;

public static final double kIndexMotorSpeed = 0.30;
}
public static class IntakeConstants {
  public static final int kUpperIntakeMotorPort = 10;
  public static final int kLowerIntakeMotorPort = 11;
  public static final int kRotatingInakeMotorPort = 12;

  public static final double kUpperIntakeMotorSpeed = 0.50;
  public static final double kLowerIntakeMotorSpeed = 0.30;
  public static final int kRotatingMotorDegree = 90;
}
} 