// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmTest_IntakeAlgae_High extends Command {
  /** Creates a new ArmTest_IntakeAlgae_High. */
  private final EndEffectorSubsystem m_EndEffectorSubsystem;

  private Timer timer;

  private boolean hasAlgae;
  public ArmTest_IntakeAlgae_High(EndEffectorSubsystem endEffectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_EndEffectorSubsystem = endEffectorSubsystem;

    timer = new Timer();

    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_EndEffectorSubsystem.Arm_intakeAlgae_High();
    m_EndEffectorSubsystem.intakeAlgae_High_Wheel();

    hasAlgae = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_EndEffectorSubsystem.hasAlgae()) {
    //   timer.start();
    // }
    // if(timer.get() > 0.5) {
    //   hasAlgae = true;
    //   timer.reset();
    //   timer.stop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.Arm_IDLE();
    m_EndEffectorSubsystem.holdAlgae();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasAlgae;
  }
}
