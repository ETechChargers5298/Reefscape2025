// Created by Gabriel R
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralScorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralRetract extends Command {

  private CoralScorer coralScorer;
  /** Creates a new CoralOut. */
  public CoralRetract() {
    coralScorer = CoralScorer.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralScorer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralScorer.backward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralScorer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
