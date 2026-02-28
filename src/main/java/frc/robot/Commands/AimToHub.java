package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
public class AimToHub extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private PhotonVisionSubsystem p_vision;
    private GenericHID controller;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final PIDController anglePID=new PIDController(0.9, 0, 0);
    private final PIDController drivePID=new PIDController(0.4,0,0);
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(12.0);
    
    
    /*private double distanceToHub=0;
    private double distanceError=0;
    private final double distanceAprilTagToHub=0.6096; //0.923798 meters for comp 
    private final double aprilTagHeight=0.7874;
    private double aprilTagDistance=0;
    private double aprilTagRotation=0;
    private double turnAngle=0;
    private double poseAmbiguity=0;
    private double limitedForward=0;
    private double limitedTurn=0;
    private double distanceToHubXY=0;
    private double hubX=0;
    private double hubY=0;
    private double hubZ=0;
    
    private final Transform3d tagToHub=new Transform3d(
        new Translation3d(-0.6096, 0.0, 0.3048), //X: -0.6096 Y: 0 Z: 0.3048
        new Rotation3d()
    );
    private final Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.0, 0.0, 0.0), // X, Y, Z in meters
    new Rotation3d(0, 0, Math.PI/2)  // Rotated 90 degrees (left)
);
    private final double targetDistance=3; // in meters*/
      public AimToHub(CommandSwerveDrivetrain s_Swerve, PhotonVisionSubsystem p_vision, GenericHID controller) {
        this.s_Swerve = s_Swerve;
        this.p_vision = p_vision;
        this.controller = controller;
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(Units.degreesToRadians(2.0));
        drivePID.setTolerance(0.05);
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve,p_vision);
    }
    @Override
    public void execute(){
       
        double forward = -controller.getRawAxis(1) * Constants.Swerve.maxSpeed;
        double rotationOutput = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;
        double limitedForward=fowardlimit.calculate(forward);
        double limitedTurn=rotationlimit.calculate(rotationOutput);
        /*boolean targetVisible = false;
        hubX = 0;
        hubY = 0;
        turnAngle = 0;

        var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    
                    poseAmbiguity = target.getPoseAmbiguity();
                    if(poseAmbiguity<0.5){
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        var transform = target.getBestCameraToTarget();
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);
                        Pose3d robotPose = new Pose3d();
                        Pose3d hubPose = robotPose.transformBy(robotToCamera)
                          .transformBy(cameraToTarget)
                          .transformBy(tagToHub);
                         hubX=hubPose.getX();
                         hubY=hubPose.getY();
                         hubZ=hubPose.getZ();
                        aprilTagRotation=transform.getRotation().getZ();
                           if(aprilTagRotation<0){
                                aprilTagRotation+=Math.PI*2;
                            }
                        //aprilTagDistance=Math.sqrt(Math.pow(tagX,2)+Math.pow(tagY, 2)+Math.pow(tagZ, 2));//Distance formula

                        //distanceToHub=Math.sqrt(Math.pow(distanceAprilTagToHub,2)+Math.pow(aprilTagDistance, 2)-2*distanceAprilTagToHub*aprilTagDistance*Math.cos(aprilTagRotation));//Law of cosines
                        //turnAngle=Math.asin(distanceAprilTagToHub*Math.sin(aprilTagRotation)/distanceToHub);//Law sin
                        distanceToHubXY=Math.sqrt(Math.pow(hubX, 2)+Math.pow(hubY, 2));
                        turnAngle=Math.atan2(hubY, hubX)-(Math.PI/2);
                        
                        }
                }
            }
            }
        }*/
        
           if (p_vision.hasTarget()==true) {
            final double targetDistance=3; // in meters
            double turnAngle=0;
            turnAngle=p_vision.getAngleToHub();
            double distanceToHubXY=0;
            distanceToHubXY=p_vision.getDistanceToHub();
            rotationOutput=anglePID.calculate(turnAngle,0)*Constants.Swerve.maxAngularVelocity;
            forward=drivePID.calculate(distanceToHubXY, targetDistance)*Constants.Swerve.maxSpeed;
            if (anglePID.atSetpoint()) {
        rotationOutput = 0;
    }
    if (drivePID.atSetpoint()) {
        forward = 0;
    }
            rotationOutput = MathUtil.clamp(rotationOutput,-Constants.Swerve.maxAngularVelocity,Constants.Swerve.maxAngularVelocity);
        forward=MathUtil.clamp(forward,-Constants.Swerve.maxSpeed,Constants.Swerve.maxSpeed);
        double distanceError=distanceToHubXY-targetDistance;
        limitedTurn=rotationlimit.calculate(rotationOutput);
        limitedForward=fowardlimit.calculate(forward);
}else{
    /*hubX = 0; 
    hubY = 0; 
    turnAngle = 0;
    limitedTurn = 0; 
    limitedForward = 0;
    rotationlimit.reset(0); 
    fowardlimit.reset(0);*/
}
//System.out.printf("hub x: %f, hub y:%f, turnAngle: %f, rotationOutput: %f, limitedTurn: %f, limitedForward %f\n",hubX,hubY,turnAngle, rotationOutput, limitedTurn, limitedForward);

        s_Swerve.setControl(driveRequest.withVelocityY(-limitedForward).withRotationalRate(-limitedTurn));
        /* 
        SmartDashboard.putNumber("Rotation of the april tag",aprilTagRotation);
        SmartDashboard.putNumber("Finds distance to april tag", aprilTagDistance);
        SmartDashboard.putNumber("Angle to turn to the hub", Units.radiansToDegrees(rotationOutput));
        SmartDashboard.putNumber("Distance to hub", distanceToHubXY);
        SmartDashboard.putNumber("April Tag Inaccurecy",poseAmbiguity);
        SmartDashboard.putNumber("Forward Photon", limitedForward);
        SmartDashboard.putNumber("Turn photon", -limitedTurn);
        SmartDashboard.putNumber("Finding the rotation of april tag", turnAngle);*/
    }
}