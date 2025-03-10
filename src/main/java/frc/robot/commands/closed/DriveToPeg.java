// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPeg extends Command {

  Drivetrain drivetrain;
  Camera cam;
  int tagID;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  private double tagX;
  private double tagY;
  private double yTolerance;
  private double yOffset;

  /** Creates a new DriveToPeg. */
  public DriveToPeg(int tagID) {
    
    drivetrain = Drivetrain.getInstance();
    cam = Camera.getInstance();
    this.tagID = tagID;
    this.xSpeed = 0.4;
    this.ySpeed = 0.3;
    this.rotSpeed = 0.7;
    this.yTolerance = 2 * 0.0254 ; //to score 4" coral (was 0.2)
    this.yOffset = 0.0; //was 0.7 at TH

    tagX = cam.getTagPose(tagID).getX();
    tagY = cam.getTagPose(tagID).getY();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, cam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if target is centered and not reached yet
    if ((drivetrain.getPose().getY() < tagY + yTolerance || drivetrain.getPose().getY() > tagY - yTolerance) 
                && drivetrain.getPose().getX() < tagX) {
      drivetrain.setDrive(xSpeed, 0.0, 0.0);
    } 

    // if target is to the left and not reached yet
    else if(drivetrain.getPose().getY() > tagY && drivetrain.getPose().getX() < tagX){
      drivetrain.setDrive(xSpeed, -ySpeed, 0.0);
    }
    
    // if target is to the RIGHT and not reached yet
    else if(drivetrain.getPose().getY() < tagY && drivetrain.getPose().getX() < tagX){
      drivetrain.setDrive(xSpeed, ySpeed, 0.0);
    }
    
    // othewise stop!
    else {
      drivetrain.stopDrive();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      drivetrain.getPose().getY() < tagY 
      && drivetrain.getPose().getX() > tagX - yOffset 
      ;
  }
}
