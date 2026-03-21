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

    public static final double UpperMotorSpeedRpm = 2150;
    public static final double LowerMotorSpeedRpm  = 1150;
    public static final double kForward = 1.0;
    public static final double kBackward = -1.0;
    public static final double kOff = 0.0;
    
    public static final String upperMotorString = "desired UpperMotorSpeed";
    public static final String lowerMotorString = "desired LowerMotorSpeed";
  }
  
  public static class IndexerConstants {
    public static final int kIndexMotorPort = 16;
    public static final int kAgitateIndexerMotorPort = 17;
    public static final int ConveyerMotorPort = 15; 
    
    public static final double ConveyerMotorSpeed = 0.75;
    public static final double kIndexMotorSpeed = 0.75;
    public static final double kAgitateMotorSpeed = 0.75;
    public static final double kForward = -1.0;
    public static final double kBackward = 1.0;
    public static final double kOff = 1.0;
    public static final double IndexerDeadband = 0.02;

    public static final String indexerSpeedString = "Indexer Speed";
    public static final String agitaterSpeedString = "Agitater Speed";
    public static final String converyMotorString = "desired ConveyerSpeed";
  }

  public static class IntakeConstants {
    public static final int kUpperIntakeMotorPort = 10;
    public static final int kLowerIntakeMotorPort = 11;
    public static final int kRotatingInakeMotorPort = 12;
    public static final int kCANDiport = 0;

    public static final double kUpperIntakeMotorSpeed = 0.5;
    public static final double kLowerIntakeMotorSpeed = 0.5;
    public static final int kPivotMotorDegree = 100;
    public static final double kPivotMotorSpeed = 0.3;
    public static final double gearRatio = 81.0;
    public static final double AgitatorSafeAngle = -10;
  }
public static final class Swerve{
    public static final double maxSpeed = 2.195; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = Math.PI*3/2; //TODO: This must be tuned to specific robot
        
    }
  }   