// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class PrepareForScore_Coral_Auto extends Command {
  private final EndEffectorSubsystem m_EndEffector;
  private final ElevatorSubsystem m_Elevator;

  public PrepareForScore_Coral_Auto(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.m_EndEffector = endEffectorSubsystem;
    this.m_Elevator = elevatorSubsystem;
    addRequirements(m_EndEffector, m_Elevator);
  }

  @Override
  public void initialize() {
    m_EndEffector.coralL4Primitive_Arm();
    m_Elevator.prepareForScore_Coral();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_Elevator.arriveSetPoint();
  }
}
