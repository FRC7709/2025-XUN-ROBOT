// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommand;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepClimb extends Command {
  private final ClimberSubsystem m_Climber;

  private final BooleanSupplier ifClimbFunc;

  private boolean ifClimb;
  public PrepClimb(ClimberSubsystem climberSubsystem, BooleanSupplier ifClimb) {
    m_Climber = climberSubsystem;

    ifClimbFunc = ifClimb;
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ifClimb = ifClimbFunc.getAsBoolean();
    if(ifClimb)
    m_Climber.Release();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Climber.arriveSetPoint()){
      m_Climber.PrepClimb();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
