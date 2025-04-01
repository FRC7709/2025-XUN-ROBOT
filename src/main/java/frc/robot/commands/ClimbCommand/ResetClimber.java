// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ResetClimber extends Command {
  private final ClimberSubsystem m_ClimberSubsystem;

  public ResetClimber(ClimberSubsystem climberSubsystem) {
    this.m_ClimberSubsystem = climberSubsystem;
    addRequirements(m_ClimberSubsystem);
  }

  @Override
  public void initialize() {
    m_ClimberSubsystem.ResetClimber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
