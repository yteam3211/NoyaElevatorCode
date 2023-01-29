package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.   

import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.RobotButtons;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class ElevatorDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSystem m_ELEsubsystem;
  double outputrught;
  double outputLeft;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorDown(ElevatorSystem subsystem, double outputrught , double outputLeft) {
    m_ELEsubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.outputrught = outputrught;
    this.outputLeft = outputLeft;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ELEsubsystem.setOutput(outputLeft,outputrught);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ELEsubsystem.setOutput(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(m_ELEsubsystem.RightMotor.getPosition() < 5000 || m_ELEsubsystem.LeftMotor.getPosition() > -5000 ){
    //     return true;
    // }
    // else
        return false;  
  }
}


