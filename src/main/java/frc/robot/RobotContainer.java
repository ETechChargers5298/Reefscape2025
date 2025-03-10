// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.utils.DPad;
// import frc.robot.commands.auto.*;
import frc.robot.commands.basic.*;
import frc.robot.commands.closed.*;
import frc.robot.commands.complex.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.combos.AutoStraightPathToCoralScore;
import frc.robot.commands.combos.DriveDtoL4;
import frc.robot.commands.combos.ElevatorIntakeCombo;
import frc.robot.commands.combos.ElevatorJawCombo;
import frc.robot.commands.combos.ElevatorScoreCombo;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDStrip;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Ports;
import frc.robot.utils.TriggerButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private LEDStrip led;
  


  private static final XboxController driverController = new XboxController(Ports.DRIVER_CONTROLLER);
  private static final XboxController operatorController = new XboxController(Ports.OPERATOR_CONTROLLER);
  private static final XboxController sysIdController = new XboxController(2);

  Drivetrain drivetrain = Drivetrain.getInstance();
  Camera cam = Camera.getInstance();

 private SendableChooser<Command> autoChooser;
 private Command auto1 = new PathPlannerAuto("Auto1");
 private Command oneMeter = new PathPlannerAuto("one meter");
 private Command testing = new PathPlannerAuto("testing");
 private Command ERComboPath = new PathPlannerAuto("ERComboPath");
 private Command auto2 = new PathPlannerAuto("auto2");
 private Command driveDtoL4 = new DriveDtoL4();



  //AlgaeHandler algae = AlgaeHandler.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    // Configure the trigger bindings
    autoChooserInit();

    led = new LEDStrip();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Link for joystick doc: https://docs.google.com/presentation/d/1cis5OrQfkU9m38LwgAMIfmPpJAZxnIC-KnAzi0JsRao/edit#slide=id.g18d2b75b637cb431_3


    //---------- DRIVETRAIN ----------//

    //Driver - LX & LY joysticks for Translation, RX joystick for Strafing
    Drivetrain.getInstance().setDefaultCommand(new SwerveDrive(
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> -driverController.getRawAxis(4),
      () -> driverController.getAButton()
    ));

    new JoystickButton(driverController,Button.kX.value).toggleOnTrue(new ToggleFieldCentric());

    // new JoystickButton(driverController,Button.kB.value).whileTrue(new DriveToPegPID(cam.closestID, "RIGHT"));
    // new JoystickButton(driverController,Button.kX.value).whileTrue(new DriveToPegPID(cam.closestID, "LEFT"));
    // new JoystickButton(driverController,Button.kY.value).whileTrue(new DriveToPegPID(cam.closestID, "STRAIGHT"));


    //---------- CORAL INTAKE ----------//

    // Driver - RT - Move Elevator in position to Intake + Spin Intake wheels
    new TriggerButton(driverController, 3).whileTrue(new ElevatorIntakeCombo());        //RT 
  

    // Operator - LT - Intake Coral with sensors
    new TriggerButton(operatorController, 2).whileTrue(new CoralInSafe());        //LT    
    
    // Operator - RB - Retract Coral if hanging too far out
    //new TriggerButton(operatorController, 2).whileTrue(new CoralRetract());       //LT
     new JoystickButton(operatorController, Button.kLeftBumper.value).whileTrue(new CoralRetract()); //LB
     


    //---------- ELEVATOR ----------//

    //Driver - Y - Elevator to Intake Height
    new JoystickButton(driverController, Button.kY.value).whileTrue(new ElevatorSetPosition(ElevatorConstants.INTAKE_HEIGHT));

    //Operator - LY joystick - Manually move Elevator
    Elevator.getInstance().setDefaultCommand(new SafeElevatorJoystick(
      () -> operatorController.getRawAxis(1)
    ));

    //Operator - DPAD - Elevator to L1, L2, L3, L4 heights
    new DPad(operatorController,180).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L1));
    new DPad(operatorController,270).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L2));
    new DPad(operatorController,0).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L3));
    new DPad(operatorController,90).whileTrue(new ElevatorSetPosition(ElevatorConstants.ELEVATOR_L4));
    

    //---------- CORAL SCORE ----------//
    
    //Operator - RT - Score Coral
    new TriggerButton(operatorController, 3).whileTrue(new CoralScore()); //RT
    

    //---------- ALGAE JAW ----------//
    
    // Operator - RY joystick - manually move Jaw up & down
    // new TriggerButton(driverController, 2).whileTrue(new ZeroAlgae());

    // AlgaeHandler.getInstance().setDefaultCommand(new SafeAlgaeJoystick(
    //   () -> operatorController.getRawAxis(5)
    // ));

    // Operator - A - Rotate jaw to Intake Angle
    //new JoystickButton(operatorController, Button.kA.value).whileTrue(new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE).repeatedly());

    // Operator - B - Rotate Jaw to Starting/Coral Stop Angle
    //new JoystickButton(operatorController, Button.kB.value).whileTrue(new SetJawAngle(MechConstants.JAW_CORAL_STOP_ANGLE).repeatedly());

    //new JoystickButton(operatorController, Button.kRightBumper.value).whileTrue(new SetJawAngle(MechConstants.JAW_UP_ANGLE)); //RB

    // Driver - B - Snap Elevator & Jaw to prep for Algae on L2
    //new JoystickButton(driverController, Button.kB.value).whileTrue(new ElevatorJawCombo(ElevatorConstants.ELEVATOR_ALGAE_L2));

    // Driver - DPAD - Set Jaw angles
    //new DPad(driverController, 0).whileTrue(new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE));
    //new DPad(driverController, 180).whileTrue(new SetJawAngle(MechConstants.JAW_CORAL_STOP_ANGLE));

    new DPad(driverController, 0).whileTrue(new SafeAlgaeJoystick(() -> 0.5));
    new DPad(driverController, 180).whileTrue(new SafeAlgaeJoystick(() -> -0.5));
 

    //---------- ALGAE TONGUE ----------//

    // Operator - Y - Eat the Algae
    //new JoystickButton(operatorController, Button.kY.value).whileTrue(new AlgaeEat());

    // Operator - X - Spit out the Algae
    //new JoystickButton(operatorController, Button.kX.value).whileTrue(new AlgaeSpit());

    new JoystickButton(sysIdController, Button.kX.value).whileTrue(Drivetrain.getInstance().transQ1);
    new JoystickButton(sysIdController, Button.kY.value).whileTrue(Drivetrain.getInstance().transQ2);
    new JoystickButton(sysIdController, Button.kA.value).whileTrue(Drivetrain.getInstance().transD1);
    new JoystickButton(sysIdController, Button.kB.value).whileTrue(Drivetrain.getInstance().transD2);

    new DPad(sysIdController,180).whileTrue(Drivetrain.getInstance().rotQ1);
    new DPad(sysIdController,270).whileTrue(Drivetrain.getInstance().rotQ2);
    new DPad(sysIdController,90).whileTrue(Drivetrain.getInstance().rotD1);
    new DPad(sysIdController,0).whileTrue(Drivetrain.getInstance().rotD2);

    


  }

public void autoChooserInit() {

    autoChooser.setDefaultOption("one meter", oneMeter);

    autoChooser.addOption("Auto 1", auto1);
    //autoChooser.addOption("one meter", oneMeter);
    autoChooser.addOption("testing", testing);
    autoChooser.addOption("ERComboPath", ERComboPath);
    autoChooser.addOption("auto2", auto2);
    autoChooser.addOption("DrivetoDL4-old", driveDtoL4);
    autoChooser.addOption("drivetopeg", new DriveToPeg(10));
    autoChooser.addOption("straightToDL4-RED", new AutoStraightPathToCoralScore(21,4) );

    // Table for AprilTag IDs
    // 9	Red Reef C --> (Blue 22)
    // 10	Red Reef D --> (Blue 21)
    // 11	Red Reef E --> (Blue 20)
    // 22	Blue Reef C --> (Red 9)
    // 21	Blue Reef D --> (Red 10)
    // 20	Blue Reef E --> (Red 11)


}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}



