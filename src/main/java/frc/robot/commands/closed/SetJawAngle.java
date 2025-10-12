
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetJawAngle extends Command {

    private static AlgaeHandler handler;
    private PIDController controller;

    
    /** Creates a new SetJawAngle. */
    public SetJawAngle(double desiredAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        handler = AlgaeHandler.getInstance();
        // p value was 0.02 may need to change back
        controller = new PIDController(0.15, 0, 0.02);

        controller.setSetpoint(desiredAngle);
        controller.setTolerance(2);

        // addRequirements(handler);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        controller.reset();
        handler.stopPivot();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double speed = controller.calculate(handler.getAngle());
        handler.pivot(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        handler.stopPivot();
        //stop the controller from moving instead!
        controller.close();
    }

    // Returns true when the command should end.
    @Override
        public boolean isFinished() {
        return controller.atSetpoint();
    }
}