package frc.robot.utils;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class DPad extends Trigger {

    public DPad(XboxController controller, double degrees) {
        super(() -> controller.getPOV() == degrees);
    }

}