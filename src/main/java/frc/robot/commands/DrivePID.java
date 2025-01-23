// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivePID extends Command {
  private Drivetrain drivetrain;
  private double desiredDistanceMeters; 
  private double desiredStrafeMeters;
  private double desiredAngleDegrees;
  private PIDController distancePID, strafePID, anglePID;
  private double startDistance;
  private double startStrafe;
  private double startAngle;


  /** Creates a new DrivePID. */
  public DrivePID(double tDistanceMeters, double tStrafeMeters, double tAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.

    //Instantiates the drivetrain object
    drivetrain = Drivetrain.getInstance();

    //saves desired pose values
    desiredDistanceMeters = tDistanceMeters;
    desiredStrafeMeters = tStrafeMeters;
    desiredAngleDegrees = tAngleDegrees;

    //setup for Distance PID Controller
    distancePID = new PIDController(1.2, 0.0, 0.0);
    startDistance = drivetrain.getPose().getX();
    double distanceSetPoint = startDistance + desiredDistanceMeters;
    distancePID.setSetpoint(distanceSetPoint);
    distancePID.setTolerance(0.05);

    //setup for Strafe PID Controller
    strafePID = new PIDController(1.2, 0.0, 0.0);
    startStrafe = drivetrain.getPose().getY();
    double strafeSetPoint = startStrafe + desiredStrafeMeters;
    strafePID.setSetpoint(strafeSetPoint);
    strafePID.setTolerance(0.05);

    //setup for Angle PID Controller
    anglePID = new PIDController(0.005, 0.0, 0.0);
    startAngle = drivetrain.getPose().getRotation().getDegrees();
    double angleSetPoint = startAngle + desiredAngleDegrees;
    strafePID.setSetpoint(angleSetPoint);
    anglePID.setTolerance(3.0);


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stopDrive(); 
    drivetrain.resetIMU();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     //calculating the distance
    double currentDistanceX = drivetrain.getPose().getX();
    double xSpeed = distancePID.calculate(currentDistanceX);

    //TODO: calculating the strafe
    double currentDistanceY = drivetrain.getPose().getY();
    double ySpeed = strafePID.calculate(currentDistanceY);

    //TODO: calculating the angle
    double currentAngle = drivetrain.getPose().getRotation().getDegrees();
    double angleSpeed = anglePID.calculate(currentAngle);

    //calling drive function
    drivetrain.setXSpeed(ySpeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distancePID.atSetpoint() && strafePID.atSetpoint() && anglePID.atSetpoint();
  }
}