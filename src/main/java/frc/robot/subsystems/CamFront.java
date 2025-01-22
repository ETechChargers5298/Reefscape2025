// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class CamFront extends SubsystemBase {

  private AprilCam cam;
  private static CamFront instance;

  // Constructor
  private CamFront() {
    this.cam = new AprilCam(VisionConstants.FRONT_CAM_NAME);
    cam.update();
  }

  // Singleton Constructor
  public static CamFront getInstance(){
    if(instance == null){
      instance = new CamFront();
    }
      return instance;
  }
  
  // public AprilCam getCam() {
  //   return cam;
  // }

  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam.getDesiredTarget(target);
  }

  public double getXDesired(PhotonTrackedTarget target) {
    return cam.getXDesired(target);
  }

  public double getX(){
    return cam.getXBest();
  }

  public double getYDesired(PhotonTrackedTarget target) {
    return cam.getYDesired(target);
  }

  public double getY(){
    return cam.getYBest();
  }

  public double getZ(){
    return cam.getZBest();
  }
  public boolean hasTarget() {
    return cam.hasTarget();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //cam.update();
    SmartDashboard.putNumber("X", getX());
    SmartDashboard.putNumber("Y", getY());
    SmartDashboard.putNumber("Z", getZ());
    for (int i = 0; i < cam.getTargets().size(); i++) {
      SmartDashboard.putString("id" + i, cam.getTargets().get(i).toString());
    }
  }
}
