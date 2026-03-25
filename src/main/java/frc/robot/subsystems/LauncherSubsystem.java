package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class LauncherSubsystem extends SubsystemBase {
  
  private TalonFX m_UpperShootMotor = new TalonFX(Constants.LauncherConstants.UpperMotorPort);
  private TalonFX m_LowerShootMotor = new TalonFX(Constants.LauncherConstants.LowerMotorPort);
  private double updatedUpperRPM=Constants.LauncherConstants.UpperMotorSpeedRpm;
  private double updatedLowerRPM=Constants.LauncherConstants.LowerMotorSpeedRpm;
  
  public LauncherSubsystem(){
    SmartDashboard.putNumber(Constants.LauncherConstants.upperMotorString, Constants.LauncherConstants.UpperMotorSpeedRpm);
    SmartDashboard.putNumber(Constants.LauncherConstants.lowerMotorString, Constants.LauncherConstants.LowerMotorSpeedRpm);

    final TalonFXConfiguration commonConfigs = new TalonFXConfiguration()
        .withMotorOutput(
          new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.Clockwise_Positive)
          )
        .withCurrentLimits(
          new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(120))
          .withStatorCurrentLimitEnable(true)
        );
  
     final TalonFXConfiguration UpperShootMotor_configs = commonConfigs.clone()
        .withMotorOutput(
          commonConfigs.MotorOutput.clone()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        );
  
     final TalonFXConfiguration LowerShootMotor_configs = commonConfigs.clone()
        .withMotorOutput(
          commonConfigs.MotorOutput.clone()
        .withInverted(InvertedValue.Clockwise_Positive)
        );
  
      final var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // To account for fricont, add 0.1 V of static feed forward
        slot0Configs.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.33 rps per V, 1/8.33 = 0.12 vols / rotation / per second
        slot0Configs.kP = 0.11; // An error of 1 rotation per second results in 0.11 v output
        slot0Configs.kI = 0; // No output for integrated error
        slot0Configs.kD = 0; // No output for error detivative
  
      m_UpperShootMotor.getConfigurator().apply(UpperShootMotor_configs);
      m_UpperShootMotor.getConfigurator().apply(slot0Configs);
      m_LowerShootMotor.getConfigurator().apply(LowerShootMotor_configs);
      m_LowerShootMotor.getConfigurator().apply(slot0Configs);
  }
  
  @Override
  public void periodic() {
    double dashUpper = SmartDashboard.getNumber(Constants.LauncherConstants.upperMotorString, updatedUpperRPM);
    double dashLower = SmartDashboard.getNumber(Constants.LauncherConstants.lowerMotorString, updatedLowerRPM);

    if (dashUpper != updatedUpperRPM){
      updatedUpperRPM = dashUpper;
    }  
    if (dashLower != updatedLowerRPM){
      updatedLowerRPM = dashLower;
    }  
  }  
  
  
  public void adjustShootSpeed(int changeShootSpeed){
    if ((updatedUpperRPM>100) && (updatedLowerRPM>100) && (changeShootSpeed<0)) {
      // this is for decreasing speed
      updatedUpperRPM+=changeShootSpeed;
      updatedLowerRPM+=changeShootSpeed;
      SmartDashboard.putNumber(Constants.LauncherConstants.upperMotorString, updatedUpperRPM);
      SmartDashboard.putNumber(Constants.LauncherConstants.lowerMotorString, updatedLowerRPM);
    }else if((updatedUpperRPM>100) && (updatedLowerRPM>0) && (changeShootSpeed>0)){
      // this is for increasing speed
      updatedUpperRPM+=changeShootSpeed;
      updatedLowerRPM+=changeShootSpeed;
      SmartDashboard.putNumber(Constants.LauncherConstants.upperMotorString, updatedUpperRPM);
      SmartDashboard.putNumber(Constants.LauncherConstants.lowerMotorString, updatedLowerRPM);
    }  
    Double currentSpeedUpperMotor=m_UpperShootMotor.getRotorVelocity().getValue().in(RadiansPerSecond);
    if(Math.abs(currentSpeedUpperMotor) > 0.0){
      setShooterSpeed(updatedUpperRPM/60.0, updatedLowerRPM/60.0);
    }  

  }  

public double getUpperTargetRPM() {
    return updatedUpperRPM;
}    

public double getLowerTargetRPM() {
    return updatedLowerRPM;
}    
 
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  
public void setShooterSpeed(Double upperMotorSpeed, Double lowerMotorSpeed){
  m_UpperShootMotor.setControl(m_velocityVoltage.withVelocity(upperMotorSpeed));
  m_LowerShootMotor.setControl(m_velocityVoltage.withVelocity(lowerMotorSpeed));
}

public void setConveyerSpeed(Double speed){
  m_ConveyerMotor.setControl(new DutyCycleOut(speed));
}

 public void distanceShooterSpeed(double distance){
  
  if (distance>0 && distance<2.29){
    double[] dis={2.29,2.69,3.2,3.7,4.2,4.7,5.2,5.7,6.2};
  double[] top={1400,1400,2050,2075,2200,2550,2550,2850,3250};
  double[] bottom={1500,1600,1050,1075,1175,1275,1275,1400,1525};

  double upperSpeed;
  double lowerSpeed;
  double x1;
  double x2;
  double y1;
  double y2;
  double y3;
  double y4;
  double topslope;
  double bottomslope;

  // =((y2-y1)/(x2-x1))*distance + ((y1 - ((y2-y1)/(x2-x1))*x1))
  // =(slope)*distance + (b)\

  /* slope=(y2-y1)/(x2-x1))
   * b=(y1 - (slope)*x1)
   */


  if (distance>0 && distance<2.29){
     x1=dis[0];
    x2=dis[1];
    y1=top[0];
    y2=top[1];
    y3=bottom[0];
    y4=bottom[1];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);


  } else if (distance>=2.29 && distance<2.69){
    x1=dis[0];
    x2=dis[1];
    y1=top[0];
    y2=top[1];
    y3=bottom[0];
    y4=bottom[1];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=2.69 && distance<3.20){
    x1=dis[1];
    x2=dis[2];
    y1=top[1];
    y2=top[2];
    y3=bottom[1];
    y4=bottom[2];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
   setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=3.20 && distance<3.70){
    x1=dis[2];
    x2=dis[3];
    y1=top[2];
    y2=top[3];
    y3=bottom[2];
    y4=bottom[3];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=3.70 && distance<4.20){
    x1=dis[3];
    x2=dis[4];
    y1=top[3];
    y2=top[4];
    y3=bottom[3];
    y4=bottom[4];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=4.20 && distance<4.7){
    x1=dis[4];
    x2=dis[5];
    y1=top[4];
    y2=top[5];
    y3=bottom[4];
    y4=bottom[5];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=4.70 && distance<5.2){
    x1=dis[5];
    x2=dis[6];
    y1=top[5];
    y2=top[6];
    y3=bottom[5];
    y4=bottom[6];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=5.20 && distance<5.70){
    x1=dis[6];
    x2=dis[7];
    y1=top[6];
    y2=top[7];
    y3=bottom[6];
    y4=bottom[7];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=5.70 && distance<6.2){
    x1=dis[7];
    x2=dis[8];
    y1=top[7];
    y2=top[8];
    y3=bottom[7];
    y4=bottom[8];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=6.2){
    x1=dis[8];
    x2=dis[9];
    y1=top[8];
    y2=top[9];
    y3=bottom[8];
    y4=bottom[9];
    topslope=(y2-y1)/(x2-x1);
    bottomslope=(y4-y3)/(x2-x1);

    upperSpeed = (topslope*distance+(y1-(topslope*x1)))/60;
    lowerSpeed = (bottomslope*distance+(y3-(bottomslope*x1)))/60;
    setShooterSpeed(upperSpeed, lowerSpeed);
}
}
}
}