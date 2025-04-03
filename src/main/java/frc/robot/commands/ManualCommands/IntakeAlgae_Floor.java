// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeAlgae_Floor extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  public IntakeAlgae_Floor(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    m_Elevator.intakeAlgae_Floor();
    m_EndEffector.Arm_intakeAlgae_Floor();
    m_EndEffector.intakeAlgae_Floor_Wheel();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Elevator.toPrimitive();
    m_EndEffector.Arm_IDLE();
    m_EndEffector.holdAlgae(); 

    if(m_EndEffector.hasAlgae()) {
      LEDConstants.hasGamePiece = true;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.hasGamePiece = false;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffector.hasAlgae() && m_EndEffector.wheelOverCurrent();
  }
}
