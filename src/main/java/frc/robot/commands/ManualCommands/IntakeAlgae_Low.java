// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae_Low extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private boolean isArrive_EndEffector;

  public IntakeAlgae_Low(ElevatorSubsystem ElevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_ElevatorSubsystem = ElevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EndEffectorSubsystem.Arm_IDLE();

    isArrive_EndEffector = false;
    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.arrivedSetpoint() && m_EndEffectorSubsystem.canMoveUp() && !m_EndEffectorSubsystem.hasAlgae()) {
      m_ElevatorSubsystem.intakeAlgae_Low();
      isArrive_EndEffector = true;
    }

    if(m_ElevatorSubsystem.arriveSetPoint() && isArrive_EndEffector) {
      m_EndEffectorSubsystem.Arm_intakeAlgae_Low();
      m_EndEffectorSubsystem.intakeAlgae_Low_Wheel();
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.Arm_IDLE();
    m_EndEffectorSubsystem.holdAlgae();

    LEDConstants.hasGamePiece = true;
    LEDConstants.LEDFlag = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffectorSubsystem.hasAlgae();
  }
}
