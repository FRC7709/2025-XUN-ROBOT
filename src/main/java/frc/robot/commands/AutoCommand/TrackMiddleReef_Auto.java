// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;

public class TrackMiddleReef_Auto extends Command {
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem_Kraken m_SwerveSubsystem;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xMeasurement;
  private double yMeasurement;
  private double rotationMeasurement;

  private double xPidError;
  private double yPidError;
  private double rotationError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  public TrackMiddleReef_Auto(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
    // PID
    xPidController = new PIDController(PhotonConstants.xPid_Kp, PhotonConstants.xPid_Ki, PhotonConstants.xPid_Kd);
    yPidController = new PIDController(PhotonConstants.yPid_Kp, PhotonConstants.yPid_Ki, PhotonConstants.yPid_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPid_Kp, PhotonConstants.rotationPid_Ki, PhotonConstants.rotationPid_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_Reef, PhotonConstants.xPidMaxOutput_Reef);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_Reef, PhotonConstants.yPidMaxOutput_Reef);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_Reef, PhotonConstants.rotationPidMaxOutput_Reef);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Auto/State", "TrackMiddleReef_INIT");
    SmartDashboard.putBoolean("Auto/TrackMiddleReef", true);
    m_SwerveSubsystem.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PhotonVisionSubsystem.hasFrontRightTarget()) {
      // Rotation-PID calculations
      
      rotationMeasurement = m_PhotonVisionSubsystem.getRotationMeasurements_FrontRight();
      rotationError = m_PhotonVisionSubsystem.getRotationError_Reef("MiddleReef_FrontRight");
      rotationMeasurement = rotationError >= 0.5 ? rotationMeasurement : PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight;
      rotationPidOutput = rotationPidController.calculate(rotationMeasurement, PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasurement = m_PhotonVisionSubsystem.getYMeasurements_FrontRight();
      yPidError = m_PhotonVisionSubsystem.getYError_Reef("MiddleReef_FrontRight");
      yMeasurement = yPidError >= 0.02 ? yMeasurement : PhotonConstants.yPidSetPoint_MiddleReef_FrontRight;
      yPidOutput = -yPidController.calculate(yMeasurement, PhotonConstants.yPidSetPoint_MiddleReef_FrontRight);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xMeasurement = m_PhotonVisionSubsystem.getXMeasurements_FrontRight();
      xPidError = m_PhotonVisionSubsystem.getXError_Reef("MiddleReef_FrontLeft");
      xMeasurement = xPidError >= 0.02 ? xMeasurement : PhotonConstants.xPidSetPoint_MiddleReef_FrontRight;
      xPidOutput = -xPidController.calculate(xMeasurement, PhotonConstants.xPidSetPoint_MiddleReef_FrontRight);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
    }else if(m_PhotonVisionSubsystem.hasFrontLeftTarget()) {
      // Rotation-PID calculations
      rotationMeasurement = m_PhotonVisionSubsystem.getRotationMeasurements_FrontLeft();
      rotationError = m_PhotonVisionSubsystem.getRotationError_Reef("MiddleReef_FrontLeft");
      rotationMeasurement = rotationError >= 0.5 ? rotationMeasurement : PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft;
      rotationPidOutput = rotationPidController.calculate(rotationMeasurement, PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasurement = m_PhotonVisionSubsystem.getYMeasurements_FrontLeft();
      yPidError = m_PhotonVisionSubsystem.getYError_Reef("MiddleReef_FrontLeft");
      yMeasurement = (yPidError > 0.02) ? yMeasurement : PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft;
      yPidOutput = -yPidController.calculate(yMeasurement, PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xMeasurement = m_PhotonVisionSubsystem.getXMeasurements_FrontLeft();
      xPidError = m_PhotonVisionSubsystem.getXError_Reef("MiddleReef_FrontLeft");
      xMeasurement = (xPidError > 0.05) ? xMeasurement : PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft;
      xPidOutput = -xPidController.calculate(xMeasurement, PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    if(m_PhotonVisionSubsystem.isArrive_Reef("MiddleReef_FrontRight") || m_PhotonVisionSubsystem.isArrive_Reef("Middle_FrontLeft")) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
      }
    // impl

    SmartDashboard.putBoolean("isFinish", LEDConstants.arrivePosition_Base);
    SmartDashboard.putNumber("TrackMiddle/xPidOutput", xPidOutput);
    SmartDashboard.putNumber("TrackMiddle/yPidOutput", yPidOutput);
    SmartDashboard.putNumber("TrackMiddle/rotationPidOutput", rotationPidOutput);
    SmartDashboard.putNumber("TrackMiddle/xPidError", xPidError);
    SmartDashboard.putNumber("TrackMiddle/yPidError", yPidError);

    if(ElevatorConstants.arriveLevel == 1) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level1_Reef_Auto);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level1_Reef_Auto);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1_Reef_Auto);
    }else if(ElevatorConstants.arriveLevel == 2) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level2_Reef_Auto);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level2_Reef_Auto);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2_Reef_Auto);
    }

    m_SwerveSubsystem.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Auto/State", "TrackMiddleReef_END");
    SmartDashboard.putBoolean("Auto/TrackMiddleReef", false);
    m_SwerveSubsystem.drive(0, 0, 0, false);

    if(m_PhotonVisionSubsystem.isArrive_Reef("MiddleReef_FrontRight") || m_PhotonVisionSubsystem.isArrive_Reef("Middle_FrontLeft")) {
      LEDConstants.arrivePosition_Base = true;
      LEDConstants.tracking = false;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.arrivePosition_Base = false;
      LEDConstants.tracking = false;
      LEDConstants.LEDFlag = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LEDConstants.arrivePosition_Base;
  }
}
