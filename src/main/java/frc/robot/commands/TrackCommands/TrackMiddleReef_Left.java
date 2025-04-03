// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackMiddleReef_Left extends Command {
  /** Creates a new TrackMiddleReef. */
  private final PhotonVisionSubsystem m_PhotonVision;
  private final SwerveSubsystem_Kraken m_Swerve;

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

  public TrackMiddleReef_Left(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    this.m_PhotonVision = photonVisionSubsystem;
    this.m_Swerve = swerveSubsystem;

    addRequirements(m_PhotonVision, m_Swerve);
    // PID
    xPidController = new PIDController(PhotonConstants.xPid_Kp, PhotonConstants.xPid_Ki, PhotonConstants.xPid_Kd);
    yPidController = new PIDController(PhotonConstants.yPid_Kp, PhotonConstants.yPid_Ki, PhotonConstants.yPid_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPid_Kp, PhotonConstants.rotationPid_Ki, PhotonConstants.rotationPid_Kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = false;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PhotonVision.hasFrontRightTarget()) {
      // Rotation-PID calculations
      
      rotationMeasurement = m_PhotonVision.getRotationMeasurements_FrontRight();
      rotationError = m_PhotonVision.getRotationError_Reef("MiddleReef_FrontRight");
      rotationMeasurement = rotationError >= 0.5 ? rotationMeasurement : PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight;
      rotationPidOutput = rotationPidController.calculate(rotationMeasurement, PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasurement = m_PhotonVision.getYMeasurements_FrontRight();
      yPidError = m_PhotonVision.getYError_Reef("MiddleReef_FrontRight");
      yMeasurement = yPidError >= 0.02 ? yMeasurement : PhotonConstants.yPidSetPoint_MiddleReef_FrontRight;
      yPidOutput = -yPidController.calculate(yMeasurement, PhotonConstants.yPidSetPoint_MiddleReef_FrontRight);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xMeasurement = m_PhotonVision.getXMeasurements_FrontRight();
      xPidError = m_PhotonVision.getXError_Reef("MiddleReef_FrontLeft");
      xMeasurement = xPidError >= 0.02 ? xMeasurement : PhotonConstants.xPidSetPoint_MiddleReef_FrontRight;
      xPidOutput = -xPidController.calculate(xMeasurement, PhotonConstants.xPidSetPoint_MiddleReef_FrontRight);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Reef);
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    if(m_PhotonVision.isArrive_Reef("Middle_FrontLeft")) {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(0, 0, 0, false);

    if(m_PhotonVision.isArrive_Reef("MiddleReef_FrontRight") || m_PhotonVision.isArrive_Reef("MiddleReef_FrontLeft")) {
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
