// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootNet extends Command {
  /** Creates a new ShootNet_Elevator. */
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  private final BooleanSupplier ifFeedFunc;

  private boolean arriveEndEffectorPrimition;
  private boolean ifFeed;
  public ShootNet(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;

    this.ifFeedFunc = ifFeed;

    addRequirements(m_Elevator, m_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_Elevator.shootNet();
    // m_EndEffector.Arm_shootAlgae_NET();
    m_EndEffector.Arm_NET_IDLE();

    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    if(m_EndEffector.arrivedSetpoint() && m_EndEffector.canMoveUp()) {
      arriveEndEffectorPrimition = true;
    }
    if(arriveEndEffectorPrimition) {
      m_Elevator.shootNet();
      if(m_Elevator.arriveSetPoint()) {
        m_EndEffector.Arm_shootAlgae_NET();
        if(m_Elevator.arriveSetPoint() && m_EndEffector.arrivedSetpoint()) {
          LEDConstants.arrivePosition_Intake = true;
          LEDConstants.LEDFlag = true;
        }else {
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.LEDFlag = true;
        }
      }
    }
    if (ifFeed) {
      m_EndEffector.Wheel_shootAlgae_NET();
    }else {
      m_EndEffector.holdAlgae();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!m_EndEffector.hasAlgae()) LEDConstants.hasAlgae = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
