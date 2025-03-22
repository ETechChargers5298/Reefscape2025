package frc.robot;


import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import frc.robot.utils.ModuleConfig;
import frc.robot.utils.Ports;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstants {

    // Sensor Offsets for the radian difference between the physical sensor orientation and the calibrated swerve direction (from REV Hardware Client)
    public static final double FL_SENSOR_OFFSET = 0.1345434; 
    public static final double FR_SENSOR_OFFSET = 0.5278169; 
    public static final double BR_SENSOR_OFFSET = 0.6507399; 
    public static final double BL_SENSOR_OFFSET = 0.5267535;

    // Angular Offsets for the radian difference between the calibrated swerve and desired forward direction (based off REV calibration tool)
    public static final double FL_ANGULAR_OFFSET = Math.PI / 2;
    public static final double FR_ANGULAR_OFFSET = Math.PI;
    public static final double BR_ANGULAR_OFFSET = 3* Math.PI / 2;
    public static final double BL_ANGULAR_OFFSET = 0;
 
    // Determine if specific modules need to be inverted
    public static final boolean FL_INVERSION = true;
    public static final boolean FR_INVERSION = true;
    public static final boolean BR_INVERSION = false;
    public static final boolean BL_INVERSION = false;

    //Constructor to hold all of the data to configure a SwerveModule
    public static final ModuleConfig SWERVE_FL = new ModuleConfig("FL", Ports.SWERVE_DRIVE_FL, Ports.SWERVE_TURN_FL, FL_SENSOR_OFFSET, FL_ANGULAR_OFFSET, FL_INVERSION);
    public static final ModuleConfig SWERVE_FR = new ModuleConfig("FR", Ports.SWERVE_DRIVE_FR, Ports.SWERVE_TURN_FR, FR_SENSOR_OFFSET, FR_ANGULAR_OFFSET, FR_INVERSION);
    public static final ModuleConfig SWERVE_BL = new ModuleConfig("BL", Ports.SWERVE_DRIVE_BL, Ports.SWERVE_TURN_BL, BL_SENSOR_OFFSET, BL_ANGULAR_OFFSET, BL_INVERSION);
    public static final ModuleConfig SWERVE_BR = new ModuleConfig("BR", Ports.SWERVE_DRIVE_BR, Ports.SWERVE_TURN_BR, BR_SENSOR_OFFSET, BR_ANGULAR_OFFSET, BR_INVERSION);

    // Distance between centers of left and right wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);

    // Distance between centers of front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25);

    // Distance from center of any wheel to center of robot geometry
    public static final double DISTANCE_TO_CENTER = Math.sqrt(Math.pow(WHEEL_BASE/2, 2) + Math.pow(TRACK_WIDTH/2, 2));

    //Bumper thickness
    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); //0.0635 meters 


    // public static final double FREE_SPIN_METER = 5.28; //???

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    // Is NavX rotation values backwards?
    public static final boolean TURN_INVERSION = false;

    // Driving Parameters - max speeds allowed, not capable
    public static final double TOP_SPEED = 4.0; //9.6
    public static final double TOP_ANGULAR_SPEED = 2 * Math.PI;
  
    // //Slew stuff from Rev
    // public static final double kDirectionSlewRate = 1; // radians per second
    // public static final double kMagnitudeSlewRate = 1.4; // percent per second (1 = 100%)
    // public static final double kRotationalSlewRate = 1; // percent per second (1 = 100%)

  }

  public static final class SwerveAutoConstants {

    //PID constants for Swerve
    public static final double TRANSLATE_P = 1;
    public static final double TRANSLATE_I = 0;
    public static final double TRANSLATE_D = 0;

    public static final double X_P = 1;
    public static final double X_I = 0;
    public static final double X_D = 0;
    public static final double X_TOL = Units.inchesToMeters(0.5); //0.025m

    public static final double Y_P = 1.0;
    public static final double Y_I = 0;
    public static final double Y_D = 0;
    public static final double Y_TOL = Units.inchesToMeters(0.5); //0.012m

    public static final double TURN_P = 0.007;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.0;
    public static final double TURN_TOL = 0.5; //0.5 degrees = 0.0087266 radians
    public static final double TURN_DERIV_TOL = 1.0; //from 2022


    // public static final double kMaxSpeedMetersPerSecond = 3;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    // public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    // public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // public static final double kPXController = 1;
    // public static final double kPYController = 1;
    // public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    //     kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    //Old Drivetrain PID constants
    //public static final PIDF TURN_PID = new PIDF(0.16, 0, 2 * Math.PI, -1, 1, true);<-- from ???
    // public static final PIDConstants translationPID = new PIDConstants(0.05, 0, 0); <-- from Rev
    // public static final PIDConstants rotationPID = new PIDConstants(0.08, 0, 0); <-- from Rev
    
    // from Rev SwerveModule Constants
    // public static final double kDrivingP = 0.03;
    // public static final double kDrivingI = 0;
    // public static final double kDrivingD = 0;
    // public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    // public static final double kDrivingMinOutput = -1;
    // public static final double kDrivingMaxOutput = 1;

    // public static final double kTurningP = 1;
    // public static final double kTurningI = 0;
    // public static final double kTurningD = 0;
    // public static final double kTurningFF = 0;
    // public static final double kTurningMinOutput = -1;
    // public static final double kTurningMaxOutput = 1;


  }

  
  public static final class SwerveModuleConstants {
  
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final double DRIVE_MOTOR_PINION_GEAR_TEETH = 14;
    public static final double SPUR_GEAR_TEETH = 20; //22 teeth, but has a diameter used for a normal 22t gear

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3); //0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // 14T NEO pinion spins 20T spur gear --> 15 tooth bevel pinion spins 45T bevel gear on wheel
    public static final double DRIVE_GEAR_RATIO = (DRIVE_MOTOR_PINION_GEAR_TEETH / SPUR_GEAR_TEETH ) * (15.0/45.0);
    public static final double DRIVE_GEAR_REDUCTION = 1/DRIVE_GEAR_RATIO; //4.286
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * WHEEL_CIRCUMFERENCE_METERS) / DRIVE_GEAR_REDUCTION;

    public static final double kDrivingEncoderPositionFactor = (WHEEL_CIRCUMFERENCE_METERS) / DRIVE_GEAR_REDUCTION; // meters
    public static final double kDrivingEncoderVelocityFactor = ((WHEEL_CIRCUMFERENCE_METERS) / DRIVE_GEAR_REDUCTION) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 40; // amps

  }


  public static class ElevatorConstants {
    
    public static final double ELEVATOR_MIN = -3;
    public static final double ELEVATOR_MAX = 100;

    public static final double INTAKE_HEIGHT = 2.5;
    public static final double ELEVATOR_L1 = 3;
    public static final double ELEVATOR_L2 = 11.7;
    public static final double ELEVATOR_L3 = 26;
    public static final double ELEVATOR_L4 = 46;
 
    public static final double ELEVATOR_PROCESSOR = 0;
    public static final double ELEVATOR_ALGAE_L2 = 15;
    public static final double ELEVATOR_ALGAE_L3 = 25;

    public static final boolean RIGHT_ELEVATOR_INVERTED = false;
    

  }
  public static class MechConstants{

    
    public static final int ENCODER_TICKS = 8192; //Counts per Revolution

    // Mech Motor Speeds for Buttons
    public static double CORAL_INTAKE_SPEED = 0.27;
    public static double CORAL_RETRACT_SPEED = 0.3;
    public static double CORAL_SCORE_SPEED = 0.5;
    public static double ALGAE_INTAKE_SPEED = 1.0;

    // Jaw Angles
    public static final double JAW_STARTING_ANGLE = 260;
    public static final double JAW_INTAKE_ANGLE = 7.21;
    // public static final double JAW_UP_ANGLE = 25;
    public static final double JAW_AUTO_ANGLE = 80;

  }

  public static class RobotConstants{
    public static final Mass MASS = Kilograms.of(63); //was 25kg = 55lbs, 63kg =140lb
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(5); //was 0.2, Typical FRC robot will be between 3-8 Kg*m^2
  }

  public static class VisionConstants{

    //Camera Name
    public static final String FRONT_CAM_NAME = "Arducam_OV9782_USB_Camera"; //"Arducam_OV9782_USB_Camera";

    // The layout of the AprilTags on the field
    // public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  }

  


}