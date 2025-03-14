package frc.robot.commands.combos;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechConstants;
import frc.robot.commands.closed.ElevatorSetPosition;
import frc.robot.commands.closed.SetJawAngle;


public class ElevatorJawCombo extends ParallelCommandGroup {
  /** Creates a new ElevatorJawCombo. */

  public ElevatorJawCombo(double position) {
    
    addCommands(
      new ElevatorSetPosition(position),
      new SetJawAngle(MechConstants.JAW_INTAKE_ANGLE)
    );

  }
}
