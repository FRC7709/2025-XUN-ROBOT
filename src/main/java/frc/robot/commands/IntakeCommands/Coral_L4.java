// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L4 extends Command {
  /** Creates a new Coral_L4_Elevator. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private final BooleanSupplier ifFeedFunc;

  private boolean ifFeed;
  public Coral_L4(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    this.ifFeedFunc = ifFeed;
    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EndEffectorSubsystem.coralL4Primitive_Arm();


    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    // 
    if(m_EndEffectorSubsystem.arrivedSetpoint() && m_EndEffectorSubsystem.canUp()) {
      m_ElevatorSubsystem.outCoral_L4();    
      if(m_ElevatorSubsystem.arriveSetPoint()) {
        m_EndEffectorSubsystem.Arm_shootCoral_L4();

        if(m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arrivedSetpoint()) {
          LEDConstants.arrivePosition_Intake = true;
          LEDConstants.LEDFlag = true;
        }else {
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.LEDFlag = true;
        }
      }
  }
    if((ifFeed) || (LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base)) {
      m_EndEffectorSubsystem.Wheel_shootCoral_L4();
    }else {
      m_EndEffectorSubsystem.stopWheel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
