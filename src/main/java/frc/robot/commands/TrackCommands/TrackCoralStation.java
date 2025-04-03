// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackCoralStation extends Command {
  /** Creates a new TrackCoralStation. */
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

  private int backLeftTarget_ID;
  private int backRightTarget_ID;


  public TrackCoralStation(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVision = photonVisionSubsystem;
    this.m_Swerve = swerveSubsystem;

    addRequirements(m_PhotonVision, m_Swerve);
    // PID
    xPidController = new PIDController(PhotonConstants.xPid_Kp, PhotonConstants.xPid_Ki, PhotonConstants.xPid_Kd);
    yPidController = new PIDController(PhotonConstants.yPid_Kp, PhotonConstants.yPid_Ki, PhotonConstants.yPid_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPid_Kp, PhotonConstants.rotationPid_Ki, PhotonConstants.rotationPid_Kd);
    // Set limits
    // xPidController.setIntegratorRange(PhotonConstants.xPidMinOutput_CoralStation, PhotonConstants.xPidMaxOutput_CoralStation);
    // yPidController.setIntegratorRange(PhotonConstants.yPidMaxOutput_CoralStation, PhotonConstants.yPidMaxOutput_CoralStation);
    // rotationPidController.setIntegratorRange(PhotonConstants.rotationPidMaxOutput_CoralStation, PhotonConstants.rotationPidMaxOutput_CoralStation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.drive(0, 0, 0, false);

    LEDConstants.tracking = true;
    LEDConstants.arrivePosition_Base = true;
    LEDConstants.LEDFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_PhotonVision.hasBackRightTarget()) {
      backRightTarget_ID = m_PhotonVision.getBackRightTargetID();

      if(backRightTarget_ID == 13 || backRightTarget_ID == 1) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackRight();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackRight);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackRight;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackRight);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVision.getYMeasurements_BackRight();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_LeftCoralStation_BackRight);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_LeftCoralStation_BackRight;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_LeftCoralStation_BackRight);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
        // X-PID calculations
        xPidMeasurements = m_PhotonVision.getXMeasurements_BackRight();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_LeftCoralStation_BackRight);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_LeftCoralStation_BackRight;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_LeftCoralStation_BackRight);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
      }else if(backRightTarget_ID == 12 || backRightTarget_ID == 2) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackRight();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_RightCoralStation_BackRight);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_RightCoralStation_BackRight;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_RightCoralStation_BackRight);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVision.getYMeasurements_BackRight();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_RightCoralStation_BackRight);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_RightCoralStation_BackRight;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_RightCoralStation_BackRight);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
        // X-PID calculations
        xPidMeasurements = m_PhotonVision.getXMeasurements_BackRight();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_RightCoralStation_BackRight);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_RightCoralStation_BackRight;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_RightCoralStation_BackRight);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
      }else {
        xPidOutput = 0;
        yPidOutput = 0;
        rotationPidOutput = 0;
      }
    }else if(m_PhotonVision.hasBackLeftTarget()) {
      backLeftTarget_ID = m_PhotonVision.getBackLeftTargetID();
      if(backLeftTarget_ID == 13 || backLeftTarget_ID == 1) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackLeft();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackLeft);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackLeft;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackLeft);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVision.getYMeasurements_BackLeft();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_LeftCoralStation_BackLeft);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_LeftCoralStation_BackLeft;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_LeftCoralStation_BackLeft);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
        // X-PID calculations
        xPidMeasurements = m_PhotonVision.getXMeasurements_BackLeft();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_LeftCoralStation_BackLeft);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_LeftCoralStation_BackLeft;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_LeftCoralStation_BackLeft);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
      }
    }else if(backLeftTarget_ID == 12 || backLeftTarget_ID == 2) {
      // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackLeft();
        rotationPidError = Math.abs(rotationPidMeasurements - PhotonConstants.rotationPidSetPoint_RightCoralStation_BackLeft);
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_RightCoralStation_BackLeft;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_RightCoralStation_BackLeft);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVision.getYMeasurements_BackLeft();
        yPidError = Math.abs(yPidMeasurements - PhotonConstants.yPidSetPoint_RightCoralStation_BackLeft);
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_RightCoralStation_BackLeft;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_RightCoralStation_BackLeft);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_CoralStation);
        // X-PID calculations
        xPidMeasurements = m_PhotonVision.getXMeasurements_BackLeft();
        xPidError = Math.abs(xPidMeasurements - PhotonConstants.xPidSetPoint_RightCoralStation_BackLeft);
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_RightCoralStation_BackLeft;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_RightCoralStation_BackLeft);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_CoralStation);
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }

    if(m_PhotonVision.isArrive_CoralStation("BackRight", "RightCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackRight", "LeftCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackLeft", "RightCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackLeft", "LeftCoralStation")) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
    }
    // impl
    if(ElevatorConstants.arriveLevel == 1) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level1_CoralStation);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level1_CoralStation);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1_CoralStation);
    }else if(ElevatorConstants.arriveLevel == 2) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level2_CoralStation);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level2_CoralStation);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2_CoralStation);
    }
    m_Swerve.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(0, 0, 0, false);

    if(m_PhotonVision.isArrive_CoralStation("BackRight", "RightCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackRight", "LeftCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackLeft", "RightCoralStation") 
    || m_PhotonVision.isArrive_CoralStation("BackLeft", "LeftCoralStation")) {
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
