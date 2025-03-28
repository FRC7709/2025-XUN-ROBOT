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
public class ShootProcessor extends Command {
  /** Creates a new ShootProcessor_Elevator. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private final BooleanSupplier ifFeedFunc;

  private boolean ifFeed;
  public ShootProcessor(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, BooleanSupplier ifFeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    this.ifFeedFunc = ifFeed;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.shootProcessor();
    m_EndEffectorSubsystem.Arm_shootAlgae_Processor();

    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();

    if(ifFeed || (LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base)) {
      m_EndEffectorSubsystem.Wheel_shootAlgae_Processor();
    }
    if(m_EndEffectorSubsystem.arrivedSetpoint()) {
      LEDConstants.arrivePosition_Intake = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.LEDFlag = true;
    }

    // if (m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arrivedSetpoint() && ifFeed) {
    //   m_EndEffectorSubsystem.Wheel_shootAlgae_Processor();
  // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.toPrimitive();
    m_EndEffectorSubsystem.Arm_IDLE();
    m_EndEffectorSubsystem.stopWheel();

    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.intakeArriving = false;
    LEDConstants.LEDFlag = true;
    if(!m_EndEffectorSubsystem.hasAlgae()) LEDConstants.hasAlgae = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
