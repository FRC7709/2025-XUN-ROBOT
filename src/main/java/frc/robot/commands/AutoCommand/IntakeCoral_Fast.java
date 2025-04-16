// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeCoral_Fast extends Command {
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  public IntakeCoral_Fast(EndEffectorSubsystem endEffectorSubsystem) {
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Auto/State", "IntakeCoral_Fast_INIT");
    m_EndEffectorSubsystem.intakeCoral_Arm();
    m_EndEffectorSubsystem.intakeCoral_Wheel();

    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Auto/State", "IntakeCoral_Fast_END");
    if(m_EndEffectorSubsystem.hasCoral()) {
      m_EndEffectorSubsystem.Arm_IDLE();
      m_EndEffectorSubsystem.stopWheel();

      LEDConstants.intakeGamePiece = false;
      LEDConstants.hasGamePiece = true;
      LEDConstants.LEDFlag = true;
    }else if(m_EndEffectorSubsystem.shouldCoralSlow()) {
      LEDConstants.intakeGamePiece = true;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;

      m_EndEffectorSubsystem.intakeCoral_Arm();
      m_EndEffectorSubsystem.stopWheel();
    }else {
      LEDConstants.intakeGamePiece = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffectorSubsystem.shouldCoralSlow() || m_EndEffectorSubsystem.hasCoral();
  }
}
