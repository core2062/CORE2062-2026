package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

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
  
  
      SmartDashboard.putNumber(Constants.LauncherConstants.upperMotorString, Constants.LauncherConstants.UpperMotorSpeedRpm);
      SmartDashboard.putNumber(Constants.LauncherConstants.lowerMotorString, Constants.LauncherConstants.LowerMotorSpeedRpm);
      SmartDashboard.putNumber(Constants.LauncherConstants.distanceString, 3);
  
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

    if (m_LowerShootMotor.get() > 0.1) {
      distanceShooterSpeed(SmartDashboard.getNumber("Distance to hub", 3.0));
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

public void distanceShooterSpeed(double distance){
  double[][] table = {  {0.0, 1400,1500},
                        {2.29,1400,1500},
                        {2.69,1400,1600}, 
                        {3.2,2050,1050},
                        {3.7,2075,1075},
                        {4.2,2200,1175},
                        {4.7,2550,1275},
                        {5.2,2300,1275},
                        {5.7,2350,1335},
                        {6.2,2400,1385},
                        {50.0, 2400,1385}};
  int index = 0;

  for (int i = 0; distance >= table[i][0] && i < table.length; i++) {
    index = i;
  }
  
  int lowerindex = index;
  int upperindex = index+1;
  
  double ratio = ((distance - table[lowerindex][0])/(table[upperindex][0] - table[lowerindex][0]));
  
  double ums = (table[lowerindex][1] + (table[upperindex][1]-table[lowerindex][1])*ratio);
  double lms = (table[lowerindex][2] + (table[upperindex][2]-table[lowerindex][2])*ratio);
  System.out.printf("distance: %f, index: %d, ums: %f, lms: %f\n", distance, index, ums, lms);

  setShooterSpeed(ums/60.0,lms/60.0);
}
  
}
   

    

