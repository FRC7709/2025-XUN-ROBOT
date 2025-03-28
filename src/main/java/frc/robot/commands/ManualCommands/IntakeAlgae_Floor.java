// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae_Floor extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private final Timer timer;

  private boolean hasAlgae;
  private boolean isStartTimer;
  public IntakeAlgae_Floor(ElevatorSubsystem ElevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = ElevatorSubsystem;
    this.m_EndEffectorSubsystem = endEffectorSubsystem;

    timer = new Timer();

    addRequirements(m_ElevatorSubsystem, m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.intakeAlgae_Floor();
    m_EndEffectorSubsystem.Arm_intakeAlgae_Floor();
    m_EndEffectorSubsystem.intakeAlgae_Floor_Wheel();

    hasAlgae = false;
    isStartTimer = false;

    // LEDConstants.intakeGamePiece = true;
    // LEDConstants.hasGamePiece = false;
    // LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_EndEffectorSubsystem.hasAlgae() && !isStartTimer) {
      timer.reset();
      timer.start();
      isStartTimer = true;

      if(timer.get() > 0.02) {
        hasAlgae = true;
      }else {
        hasAlgae = false;
      }
      
    }else {
      isStartTimer = false;
      hasAlgae = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.toPrimitive();
    m_EndEffectorSubsystem.Arm_IDLE();
    m_EndEffectorSubsystem.holdAlgae(); 
    if(m_EndEffectorSubsystem.hasAlgae()) {
      LEDConstants.hasGamePiece = true;
      LEDConstants.hasAlgae = true;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.hasGamePiece = false;
      LEDConstants.hasAlgae = false;
      LEDConstants.intakeGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_EndEffectorSubsystem.hasAlgae();
  }
}
