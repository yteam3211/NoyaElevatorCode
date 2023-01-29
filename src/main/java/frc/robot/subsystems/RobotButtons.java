package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorDown;

public class RobotButtons {
    // all joysticks:
    public static Joystick driverJoystick = new Joystick(1);
    public static Joystick coPilotJoystick = new Joystick(0);

    public Trigger elevatorUpTrigger = new Trigger(() -> coPilotJoystick.getRawButton(3));
    public Trigger elevatordownTrigger = new Trigger(() -> coPilotJoystick.getRawButton(4));

    public void loadButtons(ElevatorSystem m_elevatorSubsystem) {
        elevatordownTrigger.onTrue(new ElevatorDown(m_elevatorSubsystem, -0.1, 0.1));
        elevatorUpTrigger.onTrue(new ElevatorCommand(m_elevatorSubsystem, 0.1, -0.1));
    }
}