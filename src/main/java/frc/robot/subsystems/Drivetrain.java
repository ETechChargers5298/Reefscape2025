// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {

private static Drivetrain instance;

private final SwerveModule[] modules;
private final SwerveDriveKinematics driveKinematics;
private final SwerveDriveOdometry driveOdometry;

private final SwerveModule frontL = new SwerveModule(SwerveConstants.SWERVE_FL);
private final SwerveModule frontR = new SwerveModule(SwerveConstants.SWERVE_FR);
private final SwerveModule backL = new SwerveModule(SwerveConstants.SWERVE_BL);
private final SwerveModule backR = new SwerveModule(SwerveConstants.SWERVE_BR);

private boolean fieldCentric;

// The gyro sensor
private AHRS navX;

// Slew rate filter variables for controlling lateral acceleration
private double m_currentRotation = 0.0;
private double m_currentTranslationDir = 0.0;
private double m_currentTranslationMag = 0.0;

// private SlewRateLimiter m_magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
// private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
private double m_prevTime = WPIUtilJNI.now() * 1e-6;

//Fields that control 3 dimensions of drive motion
private double xSpeed = 0.0;
private double ySpeed = 0.0;
private double rotSpeed = 0.0;







  /** Creates a new Drivetrain. */
  private Drivetrain() {


    
    this.modules = new SwerveModule[4];
    modules[0] = frontL;
    modules[1] = frontR;
    modules[2] = backL;
    modules[3] = backR;

    this.navX = new AHRS(NavXComType.kMXP_SPI);
    
    //assign the NavX to be our sensor for rotation
    //*****no worky figure out why*******
    //this.navX = new AHRS(SPI.Port.kMXP);

    this.driveKinematics = SwerveConstants.DRIVE_KINEMATICS;

    this.driveOdometry = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getHeading(), 
      getSwerveModulePos()
    );

    fieldCentric = true;
    

  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  public void drive() {
    move(this.xSpeed, this.ySpeed, this.rotSpeed, this.fieldCentric);
  }

  //sets forward/backward motion of robot
  public void setXSpeed(double xSpeed){
    this.xSpeed = xSpeed;
  }

  //sets strafing right/left speed of robot
  public void setYSpeed(double ySpeed){
    this.ySpeed = ySpeed;
  }

  //sets rotation right/left speed of robot
  public void setRotSpeed(double rotSpeed){
    this.rotSpeed = rotSpeed;
  }





  //sets whether driving is fieldcentric or not
  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }  
  public boolean getFieldCentric() {
    return fieldCentric;
  }

  public ChassisSpeeds getSpeeds() {
    return driveKinematics.toChassisSpeeds(getModuleStates());
  }

  //method to help AutoBuilder do stuff
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    ChassisSpeeds speeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    //Store the states of each module
    SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    //cleans up any weird speeds that may be too high after kinematics equation
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    // setting the state for each module as an array
    for(int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(swerveModuleStates[i]);
    }

  }





  /**
   * Making a drive function to make the speed for drive a fraction of total
   * @author Aiden Sing
   * @param xSpeed speed of the robot front to back
   * @param ySpeed speed of robot left to right
   * @param rotSpeed speed of robot turning
   */
  public void setDrive(double xSpeed, double ySpeed, double rotSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
  }

   public void setDrive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldCentric = fieldCentric;
    //drive(xSpeed, ySpeed, rotSpeed, false);
  }

  public void stopDrive()
  {
    setDrive(0.0,0.0,0.0);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldcentric Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void move(double xSpeed, double ySpeed, double rot, boolean fieldcentric) {

    double xSpeedCommanded;
    double ySpeedCommanded;

      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveConstants.TOP_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SwerveConstants.TOP_SPEED;
    double rotSpeedDelivered = m_currentRotation * SwerveConstants.TOP_ANGULAR_SPEED;

    //var???
    //SwerveModuleState[] 
    // var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
    //     fieldcentric
    //         ? ChassisSpeeds.fromfieldcentricSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    //Store an array of speeds for each wheel
    //ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered);
    ChassisSpeeds speeds = fieldCentric ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, getHeading()) : 
      new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered);


    //Store the states of each module
    SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    //cleans up any weird speeds that may be too high after kinematics equation
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    // setting the state for each module as an array
    for(int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


    
  //---------------SWERVEMODULE HELPER METHODS --------------//

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.TOP_SPEED);
    for(int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  // method to return all the positions of the 4 modules
  public SwerveModulePosition[] getSwerveModulePos() {
    
    // return new SwerveModulePosition[] {
    //         frontLeft.getPosition(),
    //         frontRight.getPosition(),
    //         backLeft.getPosition(),
    //         backRight.getPosition()
    // };
    
    SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    for(int i = 0; i < modules.length; i++) {
      modulePosition[i] = modules[i].getPosition();
    }
    return modulePosition;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  
  
  //---------------NAVX METHODS --------------//

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    //return Rotation2d.fromDegrees(navX.getAngle());
    return navX.getRotation2d();
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

 public double getHeadingRadians() {
    return getHeading().getRadians();
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }

  // Zeroes the heading of the robot
  public void zeroHeading() {
    navX.reset();
  }
  public void resetIMU() {
    navX.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getVelocityZ() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
    //return m_gyro.getRate(IMUAxis.kZ) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public float getPitch() {
    return navX.getPitch();
  }

  public float getRoll() {
    return navX.getRoll();
  }



    //---------------ODOMETRY METHODS --------------//
  public SwerveDriveOdometry getOdometry() {

    // Pose3d p3 = new Pose3d();
    // p3.get
    // Pose2D = 
    // driveOdometry.resetPosition(getHeading(), null, getPose());
    return driveOdometry;
  }

   /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d newPose) {
    //driveOdometry.resetPosition(getHeading(), getSwerveModulePos(), newPose);
    driveOdometry.resetPosition(
        getHeading(),
        getSwerveModulePos(),
        newPose);
  }

  public void updateOdometry() {
      driveOdometry.update(
        getHeading(),
        getSwerveModulePos()
    );
  }

  // public Field2d getField() {
  //   return field;
  // }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

 public void updateTelemetry() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].updateTelemetry();
    }

    SmartDashboard.putNumber("Robot Angle", getHeading().getDegrees());

    SmartDashboard.putNumber("xOdometry", getPose().getX());
    SmartDashboard.putNumber("yOdometry", getPose().getY());
    SmartDashboard.putNumber("rotOdometry", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rotSpeed);


    //SmartDashboard.putData("Odometry Field", field);
  }












  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateTelemetry();
    drive();
  }
}