package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToTrench extends Command {

    private CommandSwerveDrivetrain s_Swerve;
    private PhotonVisionSubsystem p_vision;

    //Puts robot in robot centric mode
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    private double limitedTurn=0;
    private boolean finished = false;

    public AlignToTrench(CommandSwerveDrivetrain s_Swerve, PhotonVisionSubsystem p_vision) {
        this.s_Swerve = s_Swerve;
        this.p_vision = p_vision;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve,p_vision);
    }
    @Override
    public void execute(){
           if (p_vision.hasTrenchTarget()==true) {
                limitedTurn=p_vision.getTrenchRotation();
                finished=p_vision.atTrenchSetpoint();
           }else{
            //limitedForward=0;
            //limitedTurn=0;
           }
        //System.out.printf("raw turn: %f, raw forward: %f, limitedTurn: %f, limitedForward %f\n", rotationOutput, forward, limitedTurn, limitedForward);
        s_Swerve.setControl(driveRequest.withRotationalRate(-limitedTurn));
    }
    @Override
    public boolean isFinished() {
        return finished ;
    }
}