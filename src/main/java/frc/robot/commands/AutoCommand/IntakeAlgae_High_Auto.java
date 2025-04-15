// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeAlgae_High_Auto extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  public IntakeAlgae_High_Auto(ElevatorSubsystem ElevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = ElevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset end effector
    m_EndEffector.Arm_IDLE();
    // Led 
    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_EndEffector.arrivedSetpoint() && m_EndEffector.canMoveUp() && !m_EndEffector.hasAlgae()) {
        m_Elevator.intakeAlgae_High();

        if(m_Elevator.arriveSetPoint()) {
          m_EndEffector.Arm_intakeAlgae_High();
          m_EndEffector.intakeAlgae_High_Wheel();
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(m_EndEffector.hasAlgae()) {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.holdAlgae();

      LEDConstants.hasGamePiece = true;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }else {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.stopWheel();

      LEDConstants.intakeGamePiece = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffector.hasAlgae() && m_EndEffector.wheelOverCurrent();
  }
}
