// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NET_Elevator extends Command {
  /** Creates a new NET_Elevator. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;
  public NET_Elevator(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.`
  @Override
  public void initialize() {
    m_EndEffectorSubsystem.Arm_IDLE();

    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.intakeArriving = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.arrivedSetpoint()) {
      m_ElevatorSubsystem.shootNet();
      if(m_ElevatorSubsystem.arriveSetPoint()) {
        m_EndEffectorSubsystem.Arm_shootAlgae_NET();
        if(m_ElevatorSubsystem.arriveSetPoint() && m_EndEffectorSubsystem.arrivedSetpoint()) {
          LEDConstants.arrivePosition_Intake = true;
          LEDConstants.LEDFlag = true;
        }else {
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.LEDFlag = true;
        }
      }
    }

    if(LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base) {
      m_EndEffectorSubsystem.Wheel_shootAlgae_NET();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.tracking = false;
    LEDConstants.intakeArriving = false;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
