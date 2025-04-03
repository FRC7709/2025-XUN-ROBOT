// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnMore extends Command {
  private final EndEffectorSubsystem m_EndEffector;

  public TurnMore(EndEffectorSubsystem endEffectorSubsystem) {
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_EndEffector);
  }

  @Override
  public void initialize() {
    m_EndEffector.turnMore_Coral();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_EndEffector.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
