// Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralScorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIn extends Command {

  private CoralIntake coralIntake;
  private CoralScorer coralScorer;
  /** Creates a new CoralIn. */
  public CoralIn() {
    coralIntake = CoralIntake.getInstance();
    coralScorer = CoralScorer.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake, coralScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntake.eat();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
