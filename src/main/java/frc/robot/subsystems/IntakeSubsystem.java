package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


/*2 motors run at same time, have to configure for diff directions,
 1 button takes balls in, 1 push out, constant speed*/

 /*other motor up and down, one button,  */
//81:1

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_UpperIntakeMotor = new TalonFX(Constants.IntakeConstants.kUpperIntakeMotorPort);
    private TalonFX m_LowerIntakeMotor = new TalonFX(Constants.IntakeConstants.kLowerIntakeMotorPort);

    private TalonFX m_RotatingMotor = new TalonFX(Constants.IntakeConstants.kRotatingInakeMotorPort);
    public double gearRatio = 81.0;

    private final CANBus kCANBus = CANBus.roboRIO();
     private final CANdi candi = new CANdi(1, kCANBus);
       

public IntakeSubsystem(){
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

    final TalonFXConfiguration UpperIntakeMotor_configs = commonConfigs.clone()
        .withMotorOutput(
        commonConfigs.MotorOutput.clone()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        //TODO, check value
        );

    final TalonFXConfiguration LowerIntakeMotor_configs = commonConfigs.clone()
        .withMotorOutput(
        commonConfigs.MotorOutput.clone()
        .withInverted(InvertedValue.Clockwise_Positive)
        //TODO: check value 
        );

    final TalonFXConfiguration RotatingMotor_configs = commonConfigs.clone();     
    RotatingMotor_configs.HardwareLimitSwitch.withReverseLimitRemoteCANdiS1(candi);

// Configure PID gains (kP, kI, kD) for position control
//TODO: these configs came from LauncherSubsystem, get from SysId?
    final TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.11; // Proportional Gain
        configs.Slot0.kI = 0.0;  // Integral Gain
        configs.Slot0.kD = 0.0;  // Derivative Gain
       
// final var toApply = new CANdiConfiguration();

//     toApply.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow; 
//     toApply.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow; // This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work

//     candi.getConfigurator().apply(toApply);
    
// final var limitConfigs = new HardwareLimitSwitchConfigs();
//     limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS1;
//     limitConfigs.ReverseLimitRemoteSensorID = candi.getDeviceID();
//     limitConfigs.ReverseLimitEnable = true;
//     limitConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
// /

    m_UpperIntakeMotor.getConfigurator().apply(UpperIntakeMotor_configs);
    m_LowerIntakeMotor.getConfigurator().apply(LowerIntakeMotor_configs);
    m_RotatingMotor.getConfigurator().apply(RotatingMotor_configs);
    m_RotatingMotor.getConfigurator().apply(configs);
    // m_RotatingMotor.getConfigurator().apply(limitConfigs);
    }


public void setIntakeSpeed(Double upperMotorSpeed, Double lowerMotorSpeed){
  m_UpperIntakeMotor.setControl(new DutyCycleOut(upperMotorSpeed));
  m_LowerIntakeMotor.setControl(new DutyCycleOut(lowerMotorSpeed));
  }


/**
 * Turns the motor a relative number of degrees
  * @param degreesToTurn positive for forward, negative for backward
 */
public void turnDegrees(double degreesToTurn) {
    // Calculate rotations needed
    double rotations = (degreesToTurn / 360.0) * gearRatio;
    
    // Get current position
    double currentPos = m_RotatingMotor.getPosition().getValueAsDouble();
    
    // Create the request: current + desired
    PositionVoltage request = new PositionVoltage(currentPos + rotations);
    System.out.printf("turning, rotations %f, current pos %f\n", rotations, currentPos);  
      // Send command to motor
    m_RotatingMotor.setControl(request);
}

}
