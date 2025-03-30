// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class Coral_L1 extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  private final BooleanSupplier ifFeedFunc;
  // Variables
  private boolean ifFeed;

  public Coral_L1(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    this.ifFeedFunc = ifFeed;
    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  @Override
  public void initialize() {
    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    // Move to the position
    if(m_EndEffectorSubsystem.coralReady()) {
      m_ElevatorSubsystem.outCoral_L1();
      m_EndEffectorSubsystem.Arm_shootCoral_L1();
    }
    // Shoot when you ready
    if(ifFeed) m_EndEffectorSubsystem.Wheel_shootCoral_L1();
    else m_EndEffectorSubsystem.stopWheel();

    // LED controller
    if(m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arrivedSetpoint()) {
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
    // m_ElevatorSubsystem.toPrimitive();
    // m_EndEffectorSubsystem.primitiveArm();
    // m_EndEffectorSubsystem.stopWheel();

    // LEDConstants.intakeArriving = false;
    // LEDConstants.arrivePosition_Intake = false;
    // LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
