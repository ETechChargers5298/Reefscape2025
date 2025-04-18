package frc.robot.subsystems;


import frc.robot.utils.Ports;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.LEDStrip.SubsystemPriority;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator extends SubsystemBase {

  private static Elevator instance;

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private SparkMaxConfig leftMotorConfig;
  private SparkMaxConfig rightMotorConfig;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  private boolean ignore;


  // Elevator Constructor
  private Elevator() {

    elevatorLeftMotor = new SparkMax(Ports.ELEVATOR_LEFT_MOTOR_PORT, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(Ports.ELEVATOR_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();
    leftEncoder = elevatorLeftMotor.getEncoder();
    rightEncoder = elevatorRightMotor.getEncoder();
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    topLimitSwitch = new DigitalInput(Ports.DIGITAL_TOP_LIMIT_PORT);
    bottomLimitSwitch = new DigitalInput(Ports.DIGITAL_BOTTOM_LIMIT_PORT);

    ignore = true;
    
    rightMotorConfig.inverted(ElevatorConstants.RIGHT_ELEVATOR_INVERTED);
  
    rightMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    //leftMotorConfig.encoder.inverted();
    elevatorRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Elevator Singleton - ensures only 1 instance of Elevator is constructed
  public static Elevator getInstance() {
    if (instance == null) {
        instance = new Elevator();
    }
    return instance;
  }

  // Primary method to move the elevator up & down
  public void move(double speed){

    speed = MathUtil.clamp(speed, -0.8, 0.8);

    if(speed > 0){
      elevate(speed);
    }
    else if(speed < 0){
      descend(-speed);
    }
    else{
      stop();
    }
  }
  

  // Manually move elevator up
  public void elevate(double speed){
    //|| getTopLimit()
    if (getPosition() >= ElevatorConstants.ELEVATOR_MAX ) {
      elevatorRightMotor.set(0);
      elevatorLeftMotor.set(0);
    }

     else {
      elevatorLeftMotor.set(speed);
      elevatorRightMotor.set(-speed);
     }

  }
  
  // Manually move elevator down
  public void descend(double speed){
    //|| getBotLimit()
    if (getPosition() <= ElevatorConstants.ELEVATOR_MIN ) {
      elevatorLeftMotor.set(0);
      elevatorRightMotor.set(0);
    }

    else {
      elevatorRightMotor.set(speed);
      elevatorLeftMotor.set(-speed);
    }
    
  }  

  public void stop(){
    elevatorLeftMotor.set(0);
    elevatorRightMotor.set(0);
  }

  //----------------- SENSOR METHODS -----------------//

  // Gets the sensor position of the elevator
  public double getPosition() {
    double avg = (leftEncoder.getPosition() + -rightEncoder.getPosition()) / 2;
    return avg;
  }

  // Stops the elevator if a coral is stuck!
  public boolean coralGapStop(){
    if(!ignore() && CoralIntake.getInstance().isGapBlocked()){
      elevatorLeftMotor.set(0);
      elevatorRightMotor.set(0);
      return true;
    }
    return false;
  
  }

  // Checks if it should ignore the gap sensor if the elevator is above the intake height
  public boolean ignore() {
    if ((getPosition() > 3)) {
      return true;
    } else {
      ignore = false;
      return false;
    }
  }
  
  // Gets the value of the top limit switch
  public boolean getTopLimit() {
    return !topLimitSwitch.get();
  }

  // Gets the value of the bottom limit switch
  public boolean getBotLimit() {
    return !bottomLimitSwitch.get();
  }

  // Resets the values of both encoders
  public void resetPosition(double pos){
    leftEncoder.setPosition(pos);
    rightEncoder.setPosition(pos);
  }

  // Resets the values of both encoders to zero
  public void zeroPosition(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ignore();

    SmartDashboard.putNumber("elevator position", getPosition());
    SmartDashboard.putNumber("left elevator position", leftEncoder.getPosition());
    SmartDashboard.putNumber("right elevator position", -rightEncoder.getPosition());
    SmartDashboard.putBoolean("Top Limit", getTopLimit());
    SmartDashboard.putBoolean("Bottom Limit", getBotLimit());
    SmartDashboard.putBoolean("coral stop", coralGapStop());
    SmartDashboard.putBoolean("ignore", ignore());

    // if (instance.getBotLimit()) {
    //   LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.MIN_HEIGHT);
    // }
    
    if (instance.getPosition() > 2 && instance.getPosition() < 3.5){
      if(CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
      }

      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.INTAKE_HEIGHT);
    }
    else if (instance.getPosition() > ElevatorConstants.ELEVATOR_L1 - 1 && instance.getPosition() < ElevatorConstants.ELEVATOR_L1 + 1) {
      if(CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
      }

      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L1);
    }
    
    else if (instance.getPosition() > ElevatorConstants.ELEVATOR_L2 - 1 && instance.getPosition() < ElevatorConstants.ELEVATOR_L2 + 1) {
      if(CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
      }

      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L2);
    }

    else if (instance.getPosition() > ElevatorConstants.ELEVATOR_L3 - 1 && instance.getPosition() < ElevatorConstants.ELEVATOR_L3 + 1) {
      if(CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
      }

      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L3);
    }

    else if (instance.getPosition() > ElevatorConstants.ELEVATOR_L4 - 3 && instance.getPosition() < ElevatorConstants.ELEVATOR_L4 + 1) {
      if(CoralScorer.getInstance().hasCoral()) {
        LEDStrip.request(SubsystemPriority.ELCORAL, LEDStrip.SCORE_READY);
      }

      LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.L4);
    }

    // else if (instance.getTopLimit()) {
    //   LEDStrip.request(SubsystemPriority.ELEVATOR, LEDStrip.MAX_HEIGHT);
    // }

  }
}

//jb ms