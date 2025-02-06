// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants {
    ///(Old Robot)
    //Sensor Offsets for the radian difference between the physical sensor orientation and the calibrated swerve direction
    public static final double FL_SENSOR_OFFSET = 0.8946441; //from REV Hardware Client
    public static final double FR_SENSOR_OFFSET = 0.3716664; //from REV Hardware Client
    public static final double BR_SENSOR_OFFSET = 0.2515946; //from REV Hardware Client
    public static final double BL_SENSOR_OFFSET = 0.3813912; //from REV Hardware Client

    ///(New Robot)(Put new numbers)
    // public static final double FL_SENSOR_OFFSET = 0.7848113; //from REV Hardware Client
    // public static final double FR_SENSOR_OFFSET = 4.7082246; //from REV Hardware Client
    // public static final double BR_SENSOR_OFFSET = 1.9979531; //from REV Hardware Client
    // public static final double BL_SENSOR_OFFSET = 1.6861502; //from REV Hardware Client


    //Angular Offsets for the radian difference between the calibrated swerve and desired forward direction
    // public static final double FL_ANGULAR_OFFSET = -Math.PI/2; //Math.PI / 2; //-Math.PI / 2;
    // public static final double FR_ANGULAR_OFFSET = -Math.PI/2;
    // public static final double BR_ANGULAR_OFFSET = 3 * Math.PI/4; //Math.PI / 2;
    // public static final double BL_ANGULAR_OFFSET = Math.PI/2; //Math.PI;

    ///(New Robot)(Put new numbers)
    //Angular Offsets for the radian difference between the calibrated swerve and desired forward direction
    public static final double FL_ANGULAR_OFFSET = -Math.PI/2; //Math.PI / 2; //-Math.PI / 2;
    public static final double FR_ANGULAR_OFFSET = 2 * Math.PI/3;
    public static final double BR_ANGULAR_OFFSET = 11*Math.PI/6; //Math.PI / 2;
    public static final double BL_ANGULAR_OFFSET = 5 * Math.PI/3; //Math.PI;

    //Constructor to hold all of the data to configure a SwerveModule
    public static final ModuleConfig SWERVE_FL = new ModuleConfig("FL", Ports.SWERVE_DRIVE_FL, Ports.SWERVE_TURN_FL, FL_SENSOR_OFFSET, FL_ANGULAR_OFFSET, false);//2.9483314  +Math.PI /2);
    public static final ModuleConfig SWERVE_FR = new ModuleConfig("FR", Ports.SWERVE_DRIVE_FR, Ports.SWERVE_TURN_FR, FR_SENSOR_OFFSET, FR_ANGULAR_OFFSET, false);
    public static final ModuleConfig SWERVE_BL = new ModuleConfig("BL", Ports.SWERVE_DRIVE_BL, Ports.SWERVE_TURN_BL, BL_SENSOR_OFFSET, BL_ANGULAR_OFFSET, true); //0.6873395
    public static final ModuleConfig SWERVE_BR = new ModuleConfig("BR", Ports.SWERVE_DRIVE_BR, Ports.SWERVE_TURN_BR, BR_SENSOR_OFFSET, BR_ANGULAR_OFFSET, true);

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(26);

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26);

    public static final double DISTANCE_TO_CENTER = Math.sqrt(Math.pow(WHEEL_BASE/2, 2) + Math.pow(WHEEL_BASE/2, 2));

    // Diamter of the REV Swerve wheels in inches
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double FREE_SPIN_METER = 2;


    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));


    //TURN PID CONSTANTS
    //public static final PIDF TURN_PID = new PIDF(0.16, 0, 2 * Math.PI, -1, 1, true);
    public static final double ANGLE_THRESHOLD = Units.degreesToRadians(5);
    public static final boolean TURN_INVERSION = true;

    // Driving Parameters - max speeds allowed, not capable
    public static final double TOP_SPEED = Units.feetToMeters(2 * 4.8); //9.6
    public static final double TOP_ANGULAR_SPEED = 2 * Math.PI;
    public static final double GEER_RATTIOLI = 3.56;


    // //Slew stuff from Rev
    // public static final double kDirectionSlewRate = 1; // radians per second
    // public static final double kMagnitudeSlewRate = 1.4; // percent per second (1 = 100%)
    // public static final double kRotationalSlewRate = 1; // percent per second (1 = 100%)

    public static final boolean kGyroReversed = false;

    // public static final PIDConstants translationPID = new PIDConstants(0.05, 0, 0);
    // public static final PIDConstants rotationPID = new PIDConstants(0.08, 0, 0);

  }


public static final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth will result in a
  // robot that drives faster).
  public static final int kDrivingMotorPinionTeeth = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite direction of
  // the steering motor in the MAXSwerve Module.
  public static final boolean kTurningEncoderInverted = true;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double kNeoFreeSpeedRpm = 5676;
  public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

  public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
  public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

  public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
  public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

  public static final double kDrivingP = 0.04;
  public static final double kDrivingI = 0;
  public static final double kDrivingD = 0;
  public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  public static final double kDrivingMinOutput = -1;
  public static final double kDrivingMaxOutput = 1;

  public static final double kTurningP = 1;
  public static final double kTurningI = 0;
  public static final double kTurningD = 0;
  public static final double kTurningFF = 0;
  public static final double kTurningMinOutput = -1;
  public static final double kTurningMaxOutput = 1;

  public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  public static final int kDrivingMotorCurrentLimit = 50; // amps
  public static final int kTurningMotorCurrentLimit = 40; // amps
  }



}