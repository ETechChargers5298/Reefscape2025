//Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utils.Ports;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

public class CoralIntake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax CoralIntakeMotor;
  private SparkMaxConfig motorConfig;
  private static CoralIntake instance;
  private Timer timer;
  

  private CoralIntake() {
    CoralIntakeMotor = new SparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
  }
  public static CoralIntake getInstance(){
    if(instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }
  //
  public void eat(){
    CoralIntakeMotor.set(-MechConstants.INTAKE_SPEED);
  }
  public void spit(){
    CoralIntakeMotor.set(MechConstants.INTAKE_SPEED);
  }
  public void stop(){
    CoralIntakeMotor.set(0);
  }

  public void changeSpeed(double newSpeed){
    MechConstants.INTAKE_SPEED = newSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
