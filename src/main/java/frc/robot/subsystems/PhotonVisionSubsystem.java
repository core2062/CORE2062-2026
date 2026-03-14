package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class PhotonVisionSubsystem extends SubsystemBase{
    private double aprilTagRotation=0;
    private double turnAngle=0;
    private double poseAmbiguity=0;
    private double limitedForward=0;
    private double limitedTurn=0;
    private double distanceToHubXY=0;
    private double hubX=0;
    private double hubY=0;
    private double hubZ=0;
    private boolean targetVisible=false;
        
    private final PIDController anglePID=new PIDController(0.9, 0, 0);
    private final PIDController drivePID=new PIDController(0.4,0,0);
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(12.0);

    private double forwardOutput=0.0;
    private double rotationOutput=0.0;
    private final double targetDistance=3.9624; // in meters

    
    private final Transform3d tagToHub=new Transform3d(
        new Translation3d(-0.6096, 0.0, 0.3048), //X: -0.6096 Y: 0 Z: 0.3048
        new Rotation3d()
    );
    private final Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.0, 0, 0.0), // X, Y, Z in meters
    new Rotation3d(0, 0, Math.PI/2)  // Rotated 90 degrees (left)
);
    private final PhotonCamera camera = new PhotonCamera("regArducam2062");
    public PhotonCamera getCamera(){
        return camera;
    }
private boolean isValidId(int id) {
    return (id == 10 || id == 5 || id == 2 || id == 26 || id == 18 || id == 21);
}
    @Override 
    public void periodic(){
        targetVisible=false;
         var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            
            
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    int id=target.getFiducialId();
                    poseAmbiguity = target.getPoseAmbiguity();
                    if (isValidId(id)&&poseAmbiguity<0.5) {
                        targetVisible = true;
                        var transform = target.getBestCameraToTarget();
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
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
                        distanceToHubXY=Math.sqrt(Math.pow(hubX, 2)+Math.pow(hubY, 2));
                        turnAngle=Math.atan2(hubY, hubX)-(Math.PI/2);
                        
                        forwardOutput=drivePID.calculate(distanceToHubXY, targetDistance);
                        rotationOutput=anglePID.calculate(turnAngle,0);

                        rotationOutput = MathUtil.clamp(rotationOutput,-1,1)*Constants.Swerve.maxAngularVelocity;
                        forwardOutput=MathUtil.clamp(forwardOutput,-1,1)*Constants.Swerve.maxSpeed;

                        limitedTurn=rotationlimit.calculate(rotationOutput);
                        limitedForward=fowardlimit.calculate(forwardOutput);

                        if (anglePID.atSetpoint()) {
                            rotationOutput = 0;
                        }
                        if (drivePID.atSetpoint()) {
                            limitedForward = 0;
                        }

                        break;
                    }else{
                        turnAngle=0;
                    }
                }
            }
        }
            SmartDashboard.putNumber("Distance to hub", distanceToHubXY);
    }

    public boolean hasTarget() { 
        return targetVisible; 
    }
    public double getDistanceToHub() {
         return distanceToHubXY; 
        }
    public double getSpeedToHub(){
        return limitedForward;
    }
    public double getRotationToHub(){
        return limitedTurn;
    }
    public double getAngleToHub() { 
        return turnAngle; 
    }

}