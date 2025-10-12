// Created by Gabriel R

package frc.robot.commands.complex;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralScorer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralInSafe extends Command {

  private static CoralIntake coralIntake;
  private static CoralScorer coralScorer;
  private boolean stop;


  /** Creates a new CoralIn. */
  public CoralInSafe() {
    coralIntake = CoralIntake.getInstance();
    coralScorer = CoralScorer.getInstance();
    stop = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake, coralScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.stop();
    coralScorer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    stop = !coralIntake.isGapBlocked() && coralScorer.hasCoral();
    
    //If coral is completely inside of the scorer then were done intaking 
    if(stop){
      coralIntake.stop();
      coralScorer.spitSlow(); //???
    }
    else{
      // coralScorer.changeSpeed(MechConstants.INTAKE_SPEED);
      coralIntake.eat();
      coralScorer.spitSlow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
    coralScorer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
