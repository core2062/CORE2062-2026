package frc.robot.Commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
public class AimToHub extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private PhotonCamera camera;
    private GenericHID controller;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final PIDController anglePID=new PIDController(0.8, 0, 0);
    private final PIDController drivePID=new PIDController(0.8,0,0);
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(12.0);
    private double distanceToHub=0;
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
        new Translation3d(-0.6096, 0.0, 0.3048), 
        new Rotation3d()
    );
    private final double targetDistance=3; // in meters
      public AimToHub(CommandSwerveDrivetrain s_Swerve, PhotonCamera camera, GenericHID controller) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        this.controller = controller;
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(Units.degreesToRadians(4.0));
        drivePID.setTolerance(0.05);
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        double forward = -controller.getRawAxis(1) * Constants.Swerve.maxSpeed;
        double rotationOutput = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;
        boolean targetVisible = false;
        var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    poseAmbiguity = target.getPoseAmbiguity();
                    if(poseAmbiguity<0.4){
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        var transform = target.getBestCameraToTarget();
                        Transform3d cameraToHub = target.getBestCameraToTarget().plus(tagToHub);
                        double tagX=transform.getX();
                        double tagY=transform.getY();
                        double tagZ=transform.getZ();
                         hubX=cameraToHub.getX();
                         hubY=cameraToHub.getY();
                         hubZ=cameraToHub.getZ();
                        aprilTagRotation=transform.getRotation().getZ();

                           if(aprilTagRotation<0){
                                aprilTagRotation+=Math.PI*2;
                            }
                        aprilTagDistance=Math.sqrt(Math.pow(tagX,2)+Math.pow(tagY, 2)+Math.pow(tagZ, 2));//Distance formula

                        //distanceToHub=Math.sqrt(Math.pow(distanceAprilTagToHub,2)+Math.pow(aprilTagDistance, 2)-2*distanceAprilTagToHub*aprilTagDistance*Math.cos(aprilTagRotation));//Law of cosines
                        //turnAngle=Math.asin(distanceAprilTagToHub*Math.sin(aprilTagRotation)/distanceToHub);//Law sin
                        distanceToHubXY=Math.sqrt(Math.pow(hubX, 2)+Math.pow(hubY, 2));
                        turnAngle=Math.atan2(hubY, hubX);
                        
                        }
                }
            }
            }
        }
           if (targetVisible==true) {
            rotationOutput=anglePID.calculate(turnAngle,0)*Constants.Swerve.maxAngularVelocity;
            forward=drivePID.calculate(distanceToHubXY, targetDistance)*Constants.Swerve.maxSpeed;
            rotationOutput = MathUtil.clamp(rotationOutput,-Constants.Swerve.maxAngularVelocity,Constants.Swerve.maxAngularVelocity);
        forward=MathUtil.clamp(forward,-Constants.Swerve.maxSpeed,Constants.Swerve.maxSpeed);
        distanceError=distanceToHubXY-targetDistance;
}
if(Math.abs(turnAngle)>Units.degreesToRadians(3)){
    limitedTurn=rotationlimit.calculate(rotationOutput);
}else{
    limitedTurn=0;
}
if(Math.abs(distanceError)>0.04){
    limitedForward=fowardlimit.calculate(forward);
}else{
    limitedForward=0;
}
        s_Swerve.setControl(driveRequest.withVelocityY(-limitedForward).withRotationalRate(-limitedTurn));
        SmartDashboard.putNumber("Rotation of the april tag",aprilTagRotation);
        SmartDashboard.putNumber("Finds distance to april tag", aprilTagDistance);
        SmartDashboard.putNumber("Angle to turn to the hub", Units.radiansToDegrees(rotationOutput));
        SmartDashboard.putNumber("Distance to hub", distanceToHubXY);
        SmartDashboard.putNumber("April Tag Inaccurecy",poseAmbiguity);
        SmartDashboard.putNumber("Hub X", hubX);
        SmartDashboard.putNumber("Hub Y", hubY);
    }
}
