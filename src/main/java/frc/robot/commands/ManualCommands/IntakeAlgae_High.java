package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeAlgae_High extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  private boolean endEffectorFlag;

  public IntakeAlgae_High(ElevatorSubsystem ElevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = ElevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;

    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {
    // Reset wrist
    m_EndEffector.Arm_IDLE();

    endEffectorFlag = false;
    LEDConstants.intakeGamePiece = true;
    LEDConstants.hasGamePiece = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffector.arrivedSetpoint() && m_EndEffector.canMoveUp() && !m_EndEffector.hasAlgae()) {
      m_Elevator.intakeAlgae_High();
      endEffectorFlag = true;
    }

    if(m_Elevator.arriveSetPoint() && endEffectorFlag) {
      m_EndEffector.Arm_intakeAlgae_High();
      m_EndEffector.intakeAlgae_High_Wheel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffector.Arm_IDLE();
    m_EndEffector.holdAlgae();

    LEDConstants.hasGamePiece = true;
    LEDConstants.LEDFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffector.hasAlgae() && m_EndEffector.wheelOverCurrent();
  }
}
