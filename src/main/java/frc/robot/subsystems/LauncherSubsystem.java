package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

  private TalonFX m_ConveyerMotor = new TalonFX(Constants.LauncherConstants.ConveyerMotorPort);
 
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);


public LauncherSubsystem(){
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

   final TalonFXConfiguration ConveyerMotor_configs = commonConfigs.clone()
  .withMotorOutput(
    commonConfigs.MotorOutput.clone()
    .withInverted(InvertedValue.CounterClockwise_Positive));
  
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
    m_ConveyerMotor.getConfigurator().apply(ConveyerMotor_configs);


    SmartDashboard.putNumber("desired UpperMotorSpeed", Constants.LauncherConstants.UpperMotorSpeedRpm);
    SmartDashboard.putNumber("desired  LowerMotorSpeed", Constants.LauncherConstants.LowerMotorSpeedRpm);
    SmartDashboard.putNumber("desired ConveyerSpeed", Constants.LauncherConstants.ConveyerMotorSpeedRpm);

}

public void setShooterSpeed(Double upperMotorSpeed, Double lowerMotorSpeed){
  m_UpperShootMotor.setControl(m_velocityVoltage.withVelocity(upperMotorSpeed));
  m_LowerShootMotor.setControl(m_velocityVoltage.withVelocity(lowerMotorSpeed));
  }

public void setConveyerSpeed(Double speed){
  m_ConveyerMotor.setControl(new DutyCycleOut(speed));

  

}
}

