// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeCoral_IDLE extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;
  
  public IntakeCoral_IDLE(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    m_Elevator.intakeCoral();
    m_EndEffector.intakeCoral_Arm();
    m_EndEffector.intakeCoral_Wheel();

    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.tracking = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    if(!m_EndEffector.getFirstIR() && !m_EndEffector.getSecondIR()) {
      m_EndEffector.intakeCoralSlow_Wheel_IDLE();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_Elevator.toPrimitive();
    m_EndEffector.Arm_IDLE();
    m_EndEffector.stopWheel();

    if(m_EndEffector.hasCoral()) {
      LEDConstants.hasGamePiece = true;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.hasGamePiece = false;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_EndEffector.getFirstIR() && !m_EndEffector.getSecondIR();
  }
}
