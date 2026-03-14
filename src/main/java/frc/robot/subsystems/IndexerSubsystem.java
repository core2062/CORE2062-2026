package frc.robot.subsystems;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IndexerSubsystem extends SubsystemBase{
    private TalonFX m_IndexMotor = new TalonFX(Constants.IndexerConstants.kIndexMotorPort);
    private double indexerSpeed;
    
    public IndexerSubsystem() {
        SmartDashboard.putNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed);
        indexerSpeed = Constants.IndexerConstants.kIndexMotorSpeed;
    }

    public void setIndexerSpeed(double speed){
        m_IndexMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void periodic() {
        double dashIndexer = SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, indexerSpeed);

        if (indexerSpeed != dashIndexer){
            indexerSpeed = dashIndexer;
        }
        // System.out.printf("IndexerSubsystem: indexerSpeed is %f\n", indexerSpeed);
    }
}







