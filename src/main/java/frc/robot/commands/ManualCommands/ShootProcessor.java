// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootProcessor extends Command {
  /** Creates a new ShootProcessor_Elevator. */
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  private final BooleanSupplier ifFeedFunc;

  private boolean ifFeed;
  private boolean readyPosition;

  public ShootProcessor(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;

    ifFeedFunc = ifFeed;
    addRequirements(m_Elevator, m_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.shootProcessor();
    m_EndEffector.Arm_Algae_PreProcessor();
    m_EndEffector.holdAlgae();

    readyPosition = false;

    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    // First step
    if(Math.abs(m_EndEffector.getAngle() - WristConstants.algaePreProccesorAngle) < 3 && ifFeed) {
      m_EndEffector.Wheel_shootAlgae_Processor_Slow();
      m_EndEffector.Arm_shootAlgae_Processor();
      readyPosition = true;
    }
    // Second step
    if (readyPosition && !m_EndEffector.hasAlgae()) {
      m_EndEffector.Wheel_shootAlgae_Processor();
    }

    if(m_EndEffector.arrivedSetpoint()) {
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
    m_Elevator.toPrimitive();
    m_EndEffector.Arm_IDLE();
    m_EndEffector.stopWheel();

    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.intakeArriving = false;
    LEDConstants.LEDFlag = true;
    if(!m_EndEffector.hasAlgae()) LEDConstants.hasAlgae = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
