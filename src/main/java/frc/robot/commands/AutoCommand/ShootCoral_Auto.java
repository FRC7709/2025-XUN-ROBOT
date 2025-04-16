// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ShootCoral_Auto extends Command {
  private final EndEffectorSubsystem m_EndEffector;

  public ShootCoral_Auto(EndEffectorSubsystem endEffectorSubsystem) {
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_EndEffector);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Auto/State", "ShootCoral_Auto_INIT");
  }

  @Override
  public void execute() {
    if(LEDConstants.arrivePosition_Intake && LEDConstants.arrivePosition_Base) {
      m_EndEffector.Wheel_shootCoral_L4();
    }else {
      m_EndEffector.stopWheel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Auto/State", "ShootCoral_Auto_END");
    m_EndEffector.stopWheel();

    LEDConstants.arrivePosition_Intake = false;
    LEDConstants.tracking = false;
    LEDConstants.intakeArriving = false;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
