// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L3 extends Command {
  /** Creates a new Coral_L3_Elevator. */
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;
  private final BooleanSupplier ifFeedFunc;
  private boolean ifFeed;

  public Coral_L3(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    this.ifFeedFunc = ifFeed;

    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    // Reset the end effector
    m_EndEffector.Arm_IDLE();
    // LED
    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    // Read the ifFeed value
    ifFeed = ifFeedFunc.getAsBoolean();
    // Move to the position
    if(m_EndEffector.arrivedSetpoint() && m_EndEffector.canMoveUp()) {
      m_Elevator.outCoral_L3();
      // Move the end effector
      if(m_Elevator.arriveSetPoint()) {
        m_EndEffector.Arm_shootCoral_L3();
        // LED control
        if(m_EndEffector.arrivedSetpoint()) {
          LEDConstants.arrivePosition_Intake = true;
          LEDConstants.LEDFlag = true;
        }else {
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.LEDFlag = true;
        }
      } 
    }
    // Shoot when you ready
    if((ifFeed) || (LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base)) {
      m_EndEffector.Wheel_shootCoral_L3();
    }else {
      m_EndEffector.stopWheel();
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
