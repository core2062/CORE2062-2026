package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimToHubAuto extends Command {

    private CommandSwerveDrivetrain s_Swerve;
    private PhotonVisionSubsystem p_vision;

    //Puts robot in robot centric mode
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    private double limitedForward=0;
    private double limitedTurn=0;

    public AimToHubAuto(CommandSwerveDrivetrain s_Swerve, PhotonVisionSubsystem p_vision) {
        this.s_Swerve = s_Swerve;
        this.p_vision = p_vision;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve,p_vision);
    }
    @Override
    public void execute(){
           if (p_vision.hasTarget()==true) {
                limitedForward=p_vision.getSpeedToHub();
                limitedTurn=p_vision.getRotationToHub();
           }else{
            limitedForward=0;
            limitedTurn=0;
           }
        //System.out.printf("rotationOutput: %f, limitedTurn: %f, limitedForward %f\n", rotationOutput, limitedTurn, limitedForward);
        s_Swerve.setControl(driveRequest.withVelocityY(-limitedForward).withRotationalRate(-limitedTurn));
    }
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or is canceled
        s_Swerve.setControl(driveRequest.withVelocityX(0).withRotationalRate(0));
    }
}