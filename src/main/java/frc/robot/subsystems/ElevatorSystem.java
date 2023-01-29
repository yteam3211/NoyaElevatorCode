package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.PID.Gains;
import frc.util.dashboard.SuperShuffleBoardTab;
import frc.util.motor.SuperTalonFX;

public class ElevatorSystem extends SubsystemBase {
    public SuperTalonFX RightMotor;
    public SuperTalonFX LeftMotor;
  private DigitalInput MagnetSensor = new DigitalInput(0);
  // ShuffleboardTab tab = Shuffleboard.getTab("Tab Title");
  
  /** Creates a new ExampleSubsystem. */
  public ElevatorSystem() {
    RightMotor = new SuperTalonFX(Constants.ELEVATOR_RIGHT_MOTOR, 0, false, false, NeutralMode.Brake, new Gains("right elevator Gains", 0.05, 0, 0.020), TalonFXControlMode.Position);
    LeftMotor = new SuperTalonFX(Constants.ELEVATOR_LEFT_MOTOR, 0, false, false, NeutralMode.Brake, new Gains("left elevator Gains", 0.05, 0, 0.020), TalonFXControlMode.Position);
    // shuffleboard = new SuperShuffleBoardTab("encoder");


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean IsElevatorDown() {
    // Query some boolean state, such as a digital sensor.
    return MagnetSensor.get();
  }

  @Override
  public void periodic() {
    // System.out.println("left position: " + LeftMotor.getPosition());
    System.out.println("Right position: " + RightMotor.getPosition());
    // System.out.println(!MagnetSensor.get());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void setOutput(double outputLeft, double outputrught){
    RightMotor.set(ControlMode.PercentOutput, outputrught);
    LeftMotor.set(ControlMode.PercentOutput, outputLeft);
  }

}
