// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeAlgae_Low_Auto extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  public IntakeAlgae_Low_Auto(ElevatorSubsystem ElevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = ElevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;

    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    m_EndEffector.Arm_IDLE();

    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasAlgae = false;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
      if(m_EndEffector.arrivedSetpoint() && m_EndEffector.canMoveUp() && !m_EndEffector.hasAlgae()) {
        m_Elevator.intakeAlgae_Low();
        m_EndEffector.Arm_intakeAlgae_Low();
        m_EndEffector.intakeAlgae_Low_Wheel();
        }
    }
  

  @Override
  public void end(boolean interrupted) {
    if(m_EndEffector.hasAlgae()) {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.holdAlgae();

      LEDConstants.hasGamePiece = true;
      LEDConstants.hasAlgae = true;
      LEDConstants.LEDFlag = false;
    }else {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.stopWheel();

      LEDConstants.intakeGamePiece = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_EndEffector.hasAlgae() && m_EndEffector.wheelOverCurrent();
  }
}
