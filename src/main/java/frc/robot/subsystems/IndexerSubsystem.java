package frc.robot.subsystems;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IndexerSubsystem extends SubsystemBase{
    private TalonFX m_IndexMotor = new TalonFX(Constants.IndexerConstants.kIndexMotorPort);
    private TalonFX m_AgitateMotor = new TalonFX(Constants.IndexerConstants.kAgitateIndexerMotorPort);
    private double indexerSpeed;
    private double agitaterSpeed;
    
    public IndexerSubsystem() {
        SmartDashboard.putNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed);
        SmartDashboard.putNumber(Constants.IndexerConstants.agitaterSpeedString, Constants.IndexerConstants.kAgitateMotorSpeed);
        indexerSpeed = Constants.IndexerConstants.kIndexMotorSpeed;
        agitaterSpeed = Constants.IndexerConstants.kAgitateMotorSpeed;
    }

    public void setIndexerSpeed(double indexspeed, double agitateSpeed){
        m_IndexMotor.setControl(new DutyCycleOut(indexspeed));
        m_AgitateMotor.setControl(new DutyCycleOut(agitateSpeed));
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
    }
}







