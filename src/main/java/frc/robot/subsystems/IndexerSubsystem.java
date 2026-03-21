package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IndexerSubsystem extends SubsystemBase{
    private TalonFX m_IndexMotor = new TalonFX(Constants.IndexerConstants.kIndexMotorPort);
    private TalonFX m_AgitateMotor = new TalonFX(Constants.IndexerConstants.kAgitateIndexerMotorPort);
    private TalonFX m_ConveyerMotor = new TalonFX(Constants.IndexerConstants.ConveyerMotorPort);
    private double indexerSpeed;
    private double agitaterSpeed;
    private IntakeSubsystem m_intake;
    private double currentIndexSpeed = 0.0;
    private double currentAgitaterSpeed = 0.0;
    
    public IndexerSubsystem(IntakeSubsystem intake) {
        m_intake = intake;
        SmartDashboard.putNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed);
        SmartDashboard.putNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed);
        SmartDashboard.putNumber(Constants.IndexerConstants.converyMotorString, Constants.IndexerConstants.ConveyerMotorSpeed);
        indexerSpeed = 0.0;
        agitaterSpeed = 0.0;

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

        final TalonFXConfiguration ConveyerMotor_configs = commonConfigs.clone()
            .withMotorOutput(
            commonConfigs.MotorOutput.clone()
            .withInverted(InvertedValue.CounterClockwise_Positive));

        final TalonFXConfiguration IndexerMotor_configs = commonConfigs.clone()
            .withMotorOutput(
            commonConfigs.MotorOutput.clone()
            .withInverted(InvertedValue.Clockwise_Positive));

        final TalonFXConfiguration AgitateMotor_configs = commonConfigs.clone()
            .withMotorOutput(
            commonConfigs.MotorOutput.clone()
            .withInverted(InvertedValue.Clockwise_Positive));

         m_ConveyerMotor.getConfigurator().apply(ConveyerMotor_configs);
         m_IndexMotor.getConfigurator().apply(IndexerMotor_configs);
         m_AgitateMotor.getConfigurator().apply(AgitateMotor_configs);
}

public void setIndexerSpeed(double indexspeed, double agitateSpeed){
        m_IndexMotor.setControl(new DutyCycleOut(indexspeed));
        currentIndexSpeed = indexspeed;
        currentAgitaterSpeed = agitateSpeed;
        if (m_intake.intakeAgitatorSafe()) {
            m_AgitateMotor.setControl(new DutyCycleOut(agitateSpeed));
        } else {
            m_AgitateMotor.setControl(new DutyCycleOut(0.0));
        }
}

public void setConveyerSpeed(Double speed){
  m_ConveyerMotor.setControl(new DutyCycleOut(speed));
}

@Override
public void periodic() {
        double dashIndexer = SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, indexerSpeed);
        double dashAgitate = SmartDashboard.getNumber(Constants.IndexerConstants.agitaterSpeedString, agitaterSpeed);

        if (indexerSpeed != dashIndexer){
            indexerSpeed = dashIndexer;
        }
        if (agitaterSpeed != dashAgitate){
            agitaterSpeed = dashAgitate;
        }
        // System.out.printf("IndexerSubsystem: indexerSpeed is %f\n", indexerSpeed);
        if (!m_intake.intakeAgitatorSafe()) {
            m_AgitateMotor.setControl(new DutyCycleOut(0.0));
        } else if (Math.abs(currentIndexSpeed) > Constants.IndexerConstants.IndexerDeadband) {
            m_AgitateMotor.setControl(new DutyCycleOut(currentAgitaterSpeed));
        }
    }

}








