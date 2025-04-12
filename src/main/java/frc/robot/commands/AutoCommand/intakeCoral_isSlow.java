// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeCoral_isSlow extends Command {
  /** Creates a new intakeCoral_isSlow. */
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  public intakeCoral_isSlow(EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EndEffectorSubsystem.intakeCoral_Arm();
    m_EndEffectorSubsystem.intakeCoral_Wheel();

    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.intakeCoral_Arm();
    m_EndEffectorSubsystem.intakeCoralSlow_Wheel();

    if(m_EndEffectorSubsystem.shouldCoralSlow()) {
      LEDConstants.intakeGamePiece = true;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.intakeGamePiece = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffectorSubsystem.shouldCoralSlow();
  }
}
