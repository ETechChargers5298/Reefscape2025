package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.SetJawAngle;
import frc.robot.subsystems.AlgaeHandler;

public class ElevatorJawCombo extends ParallelCommandGroup {
  /** Creates a new ElevatorJawCombo. */
  private AlgaeHandler algae = AlgaeHandler.getInstance();

  public ElevatorJawCombo() {
    
    addCommands(
      new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE),
      algae.algaeEatCommand()
    );

  }
}
