// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommand;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class Coral_L4Test extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  private final BooleanSupplier ifFeedFunc;

  private boolean endEffectorFlag;
  private boolean ifFeed;

  public Coral_L4Test(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    this.ifFeedFunc = ifFeed;
    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    m_EndEffector.coralL4Primitive_Arm();

    endEffectorFlag = false;
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
      m_Elevator.outCoral_L4();    
      endEffectorFlag = true;
    }
    // Move the end effector
    if(m_Elevator.getPositionRot() >= ElevatorConstants.coralL2Position && endEffectorFlag) {
      m_EndEffector.Arm_shootCoral_L4();
      // LED control
      if(m_EndEffector.arrivedSetpoint()) {
        LEDConstants.arrivePosition_Intake = true;
        LEDConstants.LEDFlag = true;
      }else {
        LEDConstants.arrivePosition_Intake = false;
        LEDConstants.LEDFlag = true;
      }
    }
    // Shoot when you ready
    if((ifFeed) || (LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base)) {
      m_EndEffector.Wheel_shootCoral_L4();
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
