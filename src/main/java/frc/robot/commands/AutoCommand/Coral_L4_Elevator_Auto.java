// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_L4_Elevator_Auto extends Command {
  /** Creates a new Coral_L4_Elevator. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private boolean ifArrive_EndEffector;

  public Coral_L4_Elevator_Auto(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;
    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ElevatorSubsystem.outCoral_L4();
    // m_EndEffectorSubsystem.Arm_shootCoral_L4();
    m_EndEffectorSubsystem.coralL4Primitive_Arm();

    ifArrive_EndEffector = false;
    LEDConstants.intakeArriving = true;
    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.arrivedSetpoint() && m_EndEffectorSubsystem.canUp()) {
      m_ElevatorSubsystem.outCoral_L4();    
      ifArrive_EndEffector = true;
    }

    if(m_ElevatorSubsystem.arriveSetPoint() && ifArrive_EndEffector) {
      m_EndEffectorSubsystem.Arm_shootCoral_L4();
      LEDConstants.arrivePosition_Intake = true;
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
    if(m_ElevatorSubsystem.arriveSetPoint()) {
      LEDConstants.arrivePosition_Intake = true;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
