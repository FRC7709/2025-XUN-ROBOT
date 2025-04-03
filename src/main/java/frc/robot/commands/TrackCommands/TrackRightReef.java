// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

public class TrackRightReef extends Command {
  private final PhotonVisionSubsystem m_PhotonVision;
  private final SwerveSubsystem_Kraken m_Swerve;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xPidMeasurements;
  private double yPidMeasurements;
  private double rotationPidMeasurements;

  private double xPidError;
  private double yPidError;
  private double rotationPidError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  public TrackRightReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    this.m_PhotonVision = photonVisionSubsystem;
    this.m_Swerve = swerveSubsystem;

    addRequirements(m_PhotonVision, m_Swerve);
    // PID
    xPidController = new PIDController(PhotonConstants.xPid_Kp, PhotonConstants.xPid_Ki, PhotonConstants.xPid_Kd);
    yPidController = new PIDController(PhotonConstants.yPid_Kp, PhotonConstants.yPid_Ki, PhotonConstants.yPid_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPid_Kp, PhotonConstants.rotationPid_Ki, PhotonConstants.rotationPid_Kd);
    rotationPidController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {
    m_Swerve.drive(0, 0, 0, false);
    // LEDs
    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void execute() {
    if(m_PhotonVision.hasFrontLeftTarget()) {
      // Rotation-PID calculations
      rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_FrontLeft();
      rotationPidError = m_PhotonVision.getRotationError_Reef("FrontRight");
      rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_RightReef;
      rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_RightReef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yPidMeasurements = m_PhotonVision.getYMeasurements_FrontLeft();
      yPidError = m_PhotonVision.getYError_Reef("FrontRight");
      yPidMeasurements = (yPidError > 0.02) ? yPidMeasurements : PhotonConstants.yPidSetPoint_RightReef;
      yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_RightReef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xPidMeasurements = m_PhotonVision.getXMeasurements_FrontLeft();
      xPidError = m_PhotonVision.getXError_Reef("FrontRight");
      xPidMeasurements = (xPidError > 0.02) ? xPidMeasurements : PhotonConstants.xPidSetPoint_RightReef;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_RightReef);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);

      if(m_PhotonVision.isArrive_Reef("RightReef")) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
      }
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    SmartDashboard.putBoolean("TrackRightReef/enable", true);
    SmartDashboard.putNumber("TrackRightReef/xError", xPidError);
    SmartDashboard.putNumber("TrackRightReef/yError", yPidError);
    SmartDashboard.putNumber("TrackRightReef/rotationError", rotationPidError);

    if(ElevatorConstants.arriveLevel == 1) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level1_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level1_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1_Reef);
    }else if(ElevatorConstants.arriveLevel == 2) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level2_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level2_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2_Reef);
    }
    
    m_Swerve.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TrackRightReef/enable", false);
    // Stop the swerve
    m_Swerve.drive(0, 0, 0, false);
    // LED
    if(m_PhotonVision.isArrive_Reef("RightReef")) {
      LEDConstants.arrivePosition_Base = true;
      LEDConstants.tracking = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.arrivePosition_Base = false;
      LEDConstants.tracking = false;
      LEDConstants.LEDFlag = true;
    }
  }

  @Override
  public boolean isFinished() {
    return LEDConstants.arrivePosition_Base;
  }
}
