package frc.robot.custom_triggers;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.RobotContainer;

public class GetDpadDown extends Button {
    @Override
    public boolean get()
    {
      return RobotContainer.getDpadDown();
    }
}
