package frc.robot.subsystems;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class IndexerSubsystem extends SubsystemBase{
    private TalonFX m_IndexMotor = new TalonFX(Constants.IndexerConstants.kIndexMotorPort);

public void setIndexerSpeed(double speed){
        m_IndexMotor.setControl(new DutyCycleOut(speed));
}


    







}







