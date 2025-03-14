package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;



public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  private final SwerveModule[] modules;
  private final List<SwerveModule> modulesList;
  private final SwerveDriveKinematics driveKinematics;
  private final SwerveDriveOdometry driveOdometry;

  private final SwerveModule frontL = new SwerveModule(SwerveConstants.SWERVE_FL);
  private final SwerveModule frontR = new SwerveModule(SwerveConstants.SWERVE_FR);
  private final SwerveModule backL = new SwerveModule(SwerveConstants.SWERVE_BL);
  private final SwerveModule backR = new SwerveModule(SwerveConstants.SWERVE_BR);

  public boolean fieldCentric;

  // The gyro sensor
  public AHRS navX;

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

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field;

  // SysID Commands
  public Command transQ1;
  public Command transQ2;
  public Command transD1;
  public Command transD2;
  public Command rotQ1;
  public Command rotQ2;
  public Command rotD1;
  public Command rotD2;


  /** Drivetrain Constructor */
  private Drivetrain() {

    this.modules = new SwerveModule[4];
    modules[0] = frontL;
    modules[1] = frontR;
    modules[2] = backL;
    modules[3] = backR;

    modulesList = new ArrayList<SwerveModule>();
    modulesList.add(frontL);
    modulesList.add(frontR);
    modulesList.add(backL);
    modulesList.add(backR);
    
    //assign the NavX to be our sensor for rotation
    //*****no worky figure out why*******
    //this.navX = new AHRS(SPI.Port.kMXP);
    this.navX = new AHRS(NavXComType.kMXP_SPI);

    this.driveKinematics = SwerveConstants.DRIVE_KINEMATICS;

    this.driveOdometry = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRobotHeading(), 
      getSwerveModulePos()
    );

    this.field = new Field2d();
    this.fieldCentric = true;
    

    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    this.poseEstimator =  new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getRobotHeading(),
      getSwerveModulePos(),
      new Pose2d(),
      stateStdDevs,
      visionStdDevs);

      
    // try{
    //   RobotConfig config = RobotConfig.fromGUISettings();

    //   AutoBuilder.configure(
    //   this::getPose, 
    //   this::resetOdometry, 
    //   this::getSpeeds,
    //   (speeds, feedforwards) -> driveRobotRelative(speeds),
    //   AutoConstants.pathFollowerConfig,
    //   config, 
    //   () -> {
    //     // Boolean supplier that controls when the path will be mirrored for the red alliance
    //     // This will flip the path being followed to the red side of the field.
    //     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   this
    // );
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

    // try{
      //RobotConfig config = RobotConfig.fromGUISettings();

    // Configure AutoBuilder
    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      SwerveAutoConstants.pathFollowerConfig,
      new RobotConfig(
        MechConstants.MASS, 
        MechConstants.MOI, 
        new ModuleConfig(
          SwerveConstants.WHEEL_DIAMETER/2, 
          SwerveConstants.TOP_SPEED, 
          1.2, 
          DCMotor.getNEO(1).withReduction(ModuleConstants.kDrivingMotorReduction), 
          ModuleConstants.kDrivingMotorCurrentLimit, 
          1), 
        Constants.SwerveConstants.DRIVE_KINEMATICS.getModules()
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
    // }catch(Exception e){
    //   DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    // }
    
    // Create the SysId routines
    var translationSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        volts ->
        modulesList.forEach(
            m -> m.updateInputs(Rotation2d.fromRadians(0), volts.in(Volts))),
        // (voltage) -> this.runVolts(voltage.in(Volts)),
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );
    var rotationalSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        volts -> {
          this.frontL.updateInputs(
            Rotation2d.fromRadians((3 * Math.PI / 4) + SwerveConstants.FL_ANGULAR_OFFSET), volts.in(Volts));
        this.frontR.updateInputs(
            Rotation2d.fromRadians((Math.PI / 4) + SwerveConstants.FR_ANGULAR_OFFSET), volts.in(Volts));
        this.backL.updateInputs(
            Rotation2d.fromRadians((-3 * Math.PI / 4) + SwerveConstants.BL_ANGULAR_OFFSET), volts.in(Volts));
        this.backR.updateInputs(
            Rotation2d.fromRadians((-Math.PI / 4) + SwerveConstants.BR_ANGULAR_OFFSET), volts.in(Volts));
        },
        // (voltage) -> this.runVolts(voltage.in(Volts)),
        null, // No log consumer, since data is recorded by URCL
        this
      )
    );

    // SysID methods below return Command objects
    transQ1 = translationSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    transQ2 = translationSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    transD1 = translationSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    transD2 = translationSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

    rotQ1 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    rotQ2 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    rotD1 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    rotD2 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);

  } //ends Drivetrain constructor

  // Drivetrain Singleton - ensures only 1 instance of Drivetrain is constructed
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  // Primary method to move drivetrain -- takes in 4 fields which can be set independently, called periodically
  public void drive() {
    move(this.xSpeed, this.ySpeed, this.rotSpeed, this.fieldCentric);
  }

  // sets forward/backward motion of robot
  public void setXSpeed(double xSpeed){
    this.xSpeed = xSpeed;
  }

  // sets strafing right/left speed of robot
  public void setYSpeed(double ySpeed){
    this.ySpeed = ySpeed;
  }

  // sets rotation right/left speed of robot
  public void setRotSpeed(double rotSpeed){
    this.rotSpeed = rotSpeed;
  }

  // sets whether driving is fieldcentric or not
  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }  
  public boolean getFieldCentric() {
    return fieldCentric;
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
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, getRobotHeading()) : 
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

  // Helps AutoBuilder do stuff
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


    
  //---------------SWERVEMODULE HELPER METHODS --------------//

  public ChassisSpeeds getSpeeds() {
    return driveKinematics.toChassisSpeeds(getModuleStates());
  }

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
  public Rotation2d getRobotHeading() {
    //return Rotation2d.fromDegrees(navX.getAngle());
    return navX.getRotation2d();
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public double getRobotAngleDegrees() {
    return getRobotHeading().getDegrees();
  }

 public double getRobotAngleRadians() {
    return getRobotHeading().getRadians();
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

  public void zeroFieldAngle(){
    
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
        getRobotHeading(),
        getSwerveModulePos(),
        newPose);

    poseEstimator.resetPosition(getRobotHeading(), getSwerveModulePos(), newPose);
  }

  public void updateOdometry() {
      driveOdometry.update(
        getRobotHeading(),
        getSwerveModulePos()
    );
    poseEstimator.update(getRobotHeading(), getSwerveModulePos());
  }

  // public Field2d getField() {
  //   return field;
  // }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /**
   * Returns the currently-estimated pose of the robot relative to the FIELD
   * @return The pose.
   */
  public Pose2d getPose() {
    //return driveOdometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  public double getFieldAngleDegrees(){
    return getPose().getRotation().getDegrees();
  }

  public double getFieldAngleRadians(){
    return getPose().getRotation().getRadians();
  }

/* See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
}

/* See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }


 public void updateTelemetry() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].updateTelemetry();
    }

    SmartDashboard.putNumber("NavX Compass Heading", navX.getCompassHeading());

    SmartDashboard.putNumber("Robot Angle Degrees", getRobotAngleDegrees());
    SmartDashboard.putNumber("Robot Angle Radians", getRobotAngleRadians());
    SmartDashboard.putNumber("Field Angle Degrees", getFieldAngleDegrees());
    SmartDashboard.putNumber("Field Angle Radians", getFieldAngleRadians());
    

    SmartDashboard.putNumber("xOdometry", getPose().getX());
    SmartDashboard.putNumber("yOdometry", getPose().getY());
    SmartDashboard.putNumber("rotOdometry", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rotSpeed);

    field.setRobotPose(getPose());
    SmartDashboard.putData("Odometry Field", field);

    SmartDashboard.putBoolean("fieldCentric", fieldCentric);
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    updateTelemetry();
    drive();
    //poseEstimator.update(getHeading(), getSwerveModulePos());
  }
}