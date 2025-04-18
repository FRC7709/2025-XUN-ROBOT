// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class Coral_L1_Elevator_Auto extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  public Coral_L1_Elevator_Auto(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Auto/State", "C1_Elevator_INIT");
    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    // Move to the position
    if(m_EndEffector.canMoveUp()) {
      m_Elevator.outCoral_L1();
      m_EndEffector.Arm_shootCoral_L1();
    }
    // Shoot when you ready

    // LED controller
    if(m_Elevator.arriveSetPoint()) {
      LEDConstants.arrivePosition_Intake = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.LEDFlag = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Auto/State", "C1_Elevator_END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.arriveSetPoint() && m_EndEffector.arrivedSetpoint();
  }
}
