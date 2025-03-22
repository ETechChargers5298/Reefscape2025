package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import frc.robot.Constants.*;
import frc.robot.FieldConstants;


public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  private final SwerveModule[] modules;
  private final List<SwerveModule> modulesList;
  private final SwerveDriveKinematics driveKinematics;

  private final SwerveModule frontL = new SwerveModule(SwerveConstants.SWERVE_FL);
  private final SwerveModule frontR = new SwerveModule(SwerveConstants.SWERVE_FR);
  private final SwerveModule backL = new SwerveModule(SwerveConstants.SWERVE_BL);
  private final SwerveModule backR = new SwerveModule(SwerveConstants.SWERVE_BR);

  public boolean fieldCentric;

  // The gyro sensor
  public AHRS navX;

  public PIDController controllerNumberOfRotations = new PIDController(1.6, 0, 0.001);

  // Slew rate filter variables for controlling lateral acceleration
  // private double m_currentRotSpeed = 0.0;
  // private double m_currentTranslationDir = 0.0;
  // private double m_currentTranslationMag = 0.0;
  // private SlewRateLimiter m_magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
  // private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
  // private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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

    this.field = new Field2d();
    this.fieldCentric = false;
    
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);


    this.poseEstimator =  new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getRobotHeading(),
      getSwerveModulePos(),
      FieldConstants.getInitialPose(),
      stateStdDevs,
      visionStdDevs);

    autoConfig();
    sysIdConfig();
    /* If we're off by 0.1" on the wheel diameter, then after 4 rotations (about 1m on a 3.1" diameter wheel) we'll be off by 3.1cm or 1.24", or 7.5 degrees.
     * After 40 rotations multiply all that by 10 so ~75 degrees off. Mark the bottom of a wheel with a sharpie and run this test and see where the mark ends up.
     * Adjust the wheel diameter so that the mark ends up where it started. Test this again with different numbers of rotations, because in theory if we're
     * accurate enough, we should be able to command any number of rotations and the mark will end up at the same spot. Make sure at least some of the number of
     * commanded rotations are co-prime, to make sure we can't error that repeat with some regularity.
     */
    var numrotations = 1;
    var tolerance_deg = 2;
    controllerNumberOfRotations.setSetpoint(numrotations * Constants.SwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS);
    //controllerNumberOfRotations.setTolerance(Constants.SwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS * tolerance_deg/360);
    controllerNumberOfRotations.setTolerance(0.002);
  }

  //Path Planner Drive Controller
  public final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
    new PIDConstants(SwerveAutoConstants.TRANSLATE_P, SwerveAutoConstants.TRANSLATE_I, SwerveAutoConstants.TRANSLATE_D), // Translation constants 
    new PIDConstants(SwerveAutoConstants.TURN_P, SwerveAutoConstants.TURN_I, SwerveAutoConstants.TURN_D) // Rotation constants 
  );

  private void autoConfig(){

    // Configure AutoBuilder
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      pathFollowerConfig,
      new RobotConfig(
        RobotConstants.MASS, 
        RobotConstants.MOI, 
        new ModuleConfig(
          SwerveModuleConstants.WHEEL_DIAMETER_METERS/2, 
          SwerveConstants.TOP_SPEED, 
          1.2, 
          DCMotor.getNEO(1).withReduction(SwerveModuleConstants.DRIVE_GEAR_REDUCTION), 
          SwerveModuleConstants.kDrivingMotorCurrentLimit, 
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
   
  }

  private void sysIdConfig(){
    
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

  }


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

    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double rotSpeedCommanded = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveConstants.TOP_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SwerveConstants.TOP_SPEED;
    double rotSpeedDelivered = rotSpeedCommanded * SwerveConstants.TOP_ANGULAR_SPEED;

    //SwerveModuleState[] 
    // var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
    //     fieldcentric
    //         ? ChassisSpeeds.fromfieldcentricSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    //Store an array of speeds for each wheel. By default do robot centric speeds but if fieldCentric use fromFieldRelativeSpeeds
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered);
    if (fieldCentric) {
      var rotation = getPose().getRotation();
      var allianceOptional = DriverStation.getAlliance();
      if (allianceOptional.isPresent() && allianceOptional.get() == DriverStation.Alliance.Red) {
        // Flip the rotation if our driverstation is red alliance so that driving is "driver centric"
        rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
      }
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, rotation);
    }

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
    SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
    for(int i = 0; i < modules.length; i++) {
      SwerveModulePosition currentPos = modules[i].getPosition();
      modulePosition[i] = new SwerveModulePosition(-currentPos.distanceMeters, currentPos.angle);
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
  public void zeroRobotHeading() {
    navX.reset();
  }

  public void zeroFieldAngle(){
    
  }
  // public void resetIMU() {
  //   navX.reset();
  // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getVelocityZ() * (SwerveConstants.TURN_INVERSION ? -1.0 : 1.0);
    //return m_gyro.getRate(IMUAxis.kZ) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public float getPitch() {
    return navX.getPitch();
  }

  public float getRoll() {
    return navX.getRoll();
  }



    //---------------POSE ESTIMATION METHODS --------------//
   /**
   * Resets the poseEstimator to the specified pose.
   * @param pose The pose to which to set the poseEstimator
   */
  public void resetPose(Pose2d newPose) {
    poseEstimator.resetPosition(getRobotHeading(), getSwerveModulePos(), newPose);
  }

  public void updatePose() {
    poseEstimator.update(getRobotHeading(), getSwerveModulePos());
  }

  public Field2d getField() {
    return field;
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /**
   * Returns the currently-estimated pose of the robot relative to the FIELD
   * @return The pose.
   */
  public Pose2d getPose() {
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



 public void updateModuleTelemetry() {
    for(int i = 0; i < modules.length; i++) {
      modules[i].updateTelemetry();
    }
  }


  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    /* ===== ROTATION TESTING CODE ===== */
    /* See comment in the constructor about what this is for. Also note that I've hardcoded a particular module here,
     * so make sure to make the mark on that module, and also test the other modules.
     */
    double xspeed = controllerNumberOfRotations.calculate(frontL.getPosition().distanceMeters);
    //move(xspeed, 0, 0, false);
    setXSpeed(xspeed);
    /* ===== END ROTATION TESTING CODE ===== */

    updatePose();
    updateModuleTelemetry();
    drive();
    
    SmartDashboard.putNumber("NavX Compass Heading", navX.getCompassHeading());

    SmartDashboard.putNumber("Robot Angle Degrees", getRobotAngleDegrees());
    SmartDashboard.putNumber("Robot Angle Radians", getRobotAngleRadians());
    SmartDashboard.putNumber("Field Angle Degrees", getFieldAngleDegrees());
    SmartDashboard.putNumber("Field Angle Radians", getFieldAngleRadians());
    

    SmartDashboard.putNumber("PoseX", getPose().getX());
    SmartDashboard.putNumber("PoseY", getPose().getY());
    SmartDashboard.putNumber("PoseAngle", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rotSpeed);

    field.setRobotPose(getPose());
    SmartDashboard.putData("PoseEstimator Field", field);
    SmartDashboard.putBoolean("fieldCentric", fieldCentric);
    SmartDashboard.putNumber("FL distanceMeters", frontL.getPosition().distanceMeters);
    SmartDashboard.putNumber("BL distanceMeters", backL.getPosition().distanceMeters);
    SmartDashboard.putNumber("BR distanceMeters", backR.getPosition().distanceMeters);
    SmartDashboard.putNumber("FR distanceMeters", frontR.getPosition().distanceMeters);
    SmartDashboard.putNumber("controller error", controllerNumberOfRotations.getError());
  }
}