 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KevinVision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


  public KevinVision() {
    public final PhotonCamera camera = new PhotonCamera("KevinVision");
    private final double cameraPitchRadians = edu.wpi.first.math.util.Units.degreesToRadians(31.0);
    //Change
    private final double cameraHeightMeters = 0.51;
    private final double targetHeightMeters = 2.6416;
    private double yaw;
    private double pitch;
    private double distance;
    private boolean hasTargets;
 
   
    

  }



  


  //Methods Check what method the PhotonVISION API Provides

  //normal coding (NO methods needed)

  //METHODS UNDER HERE

  //public void getRange() {
   


  @Override
  public void periodic() {
    org.photonvision.targeting.PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
        hasTargets = true;
        org.photonvision.targeting.PhotonTrackedTarget target = result.getBestTarget();
        yaw = target.getYaw();
        pitch = target.getPitch();
        distance = PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeightMeters, 
            targetHeightMeters, 
            cameraPitchRadians, 
            edu.wpi.first.math.util.Units.degreesToRadians(pitch)
        );
    } else {
        hasTargets = false;
    }
    //SmartDashboard.putNumber("Distance", distance);

    // This method will be called once per scheduler run
    //smartdashboard.getRange("PV Range", answer);
  }

   // Method to get the yaw
   public double getYaw() {
    return -yaw;
}

// Method to get the pitch
public double getPitch() {
    return pitch;
}

// Method to get the distance 
public double getDistance() {
    return distance;
}

// Method to check whether vision has targets
public boolean hasTarget() {
    return hasTargets;
}}


  @Override //Complete useless
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

