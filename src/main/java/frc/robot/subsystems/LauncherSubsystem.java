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
  double upperSpeed;
  double lowerSpeed;
  
  if (distance>0 && distance<2.29){
    upperSpeed = 1400/60;
    lowerSpeed = ((250*distance)+927.5)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);
  
  } else if (distance>=2.29 && distance<2.69){
    upperSpeed = 1400/60;
    lowerSpeed = ((250*distance)+927.5)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=2.69 && distance<3.20){
    upperSpeed = ((1274.51*distance)-2028.43)/60;
    lowerSpeed = ((-1078.43*distance)+4500.98)/60;
   setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=3.20 && distance<3.70){
    upperSpeed = ((50*distance)+1890)/60;
    lowerSpeed = ((50*distance)+890)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=3.70 && distance<4.20){
    upperSpeed = ((250*distance)+1150)/60;
    lowerSpeed = ((200*distance)+335)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=4.20 && distance<4.7){
    upperSpeed = ((700*distance)-740)/60;
    lowerSpeed = ((200*distance)+335)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=4.70 && distance<5.2){
    upperSpeed = 2550/60;
    lowerSpeed = 1275/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=5.20 && distance<5.70){
    upperSpeed = ((600*distance)-570)/60;
    lowerSpeed = ((250*distance)-25)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=5.70 && distance<6.2){
    upperSpeed = ((50*distance)+1890)/60;
    lowerSpeed = ((50*distance)+890)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);

  } else if (distance>=6.2){
    upperSpeed = ((800*distance)-1710)/60;
    lowerSpeed = ((250*distance)-25)/60;
    setShooterSpeed(upperSpeed, lowerSpeed);
}
}
}


