// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimitiveIntake_Auto extends Command {
  /** Creates a new PrimitiveIntake_Auto. */
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;
  public PrimitiveIntake_Auto(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LEDConstants.hasAlgae) {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.holdAlgae();
      if(m_EndEffector.arrivedSetpoint()) {
        m_Elevator.toPrimitive();
      }

      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = true;
      LEDConstants.LEDFlag = true;
    }else {
      m_EndEffector.Arm_IDLE();
      m_EndEffector.stopWheel();
      if(m_EndEffector.arrivedSetpoint()) {
        m_Elevator.toPrimitive();
      }

      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.arrivePrimition() && m_EndEffector.arrivedSetpoint();
  }
}
