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
public class TrackNet extends Command {
  /** Creates a new TrackNet. */
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

  private int backRightTarget_ID;
  private int backLeftTarget_ID;

  public TrackNet(SwerveSubsystem_Kraken swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    backRightTarget_ID = m_PhotonVision.getBackRightTargetID();
    backLeftTarget_ID = m_PhotonVision.getBackLeftTargetID();
    if(m_PhotonVision.hasBackRightTarget()) {
        if(backRightTarget_ID == 20 || backRightTarget_ID == 11) {
          // Rotation-PID calculations
          rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackRight();
          rotationPidError = m_PhotonVision.getRotationError_Net("BackRight", "ID20_ID11");
          rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_Net_BackRight_ID20_ID11;
          rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Net_BackRight_ID20_ID11);
          rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Net);
          // Y-PID calculations
          yPidMeasurements = m_PhotonVision.getYMeasurements_BackRight();
          yPidError = m_PhotonVision.getYError_Net("BackRight", "ID20_ID11");
          yPidMeasurements = (yPidError > 0.02) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Net_BackRight_ID20_ID11;
          yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Net_BackRight_ID20_ID11);
          yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Net);
          // X-PID calculations
          xPidMeasurements = m_PhotonVision.getXMeasurements_BackRight();
          xPidError = m_PhotonVision.getXError_Net("BackRight", "ID20_ID11");
          xPidMeasurements = (xPidError > 0.02) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Net_BackRight_ID20_ID11;
          xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Net_BackRight_ID20_ID11);
          xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Net);
        }else if(backRightTarget_ID == 21 || backRightTarget_ID == 10) {
          // Rotation-PID calculations
          rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackRight();
          rotationPidError = m_PhotonVision.getRotationError_Net("BackRight", "ID21_ID10");
          rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Net_BackRight_ID21_ID10);
          rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Net);
          // Y-PID calculations
          yPidMeasurements = m_PhotonVision.getYMeasurements_BackRight();
          yPidError = m_PhotonVision.getYError_Net("BackRight", "ID21_ID10");
          yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Net_BackRight_ID21_ID10;
          yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Net_BackRight_ID21_ID10);
          yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Net);
          // X-PID calculations
          xPidMeasurements = m_PhotonVision.getXMeasurements_BackRight();
          xPidError = m_PhotonVision.getXError_Net("BackRight", "ID21_ID10");
          xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Net_BackRight_ID21_ID10;
          xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Net_BackRight_ID21_ID10);
          xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Net);
      }else if(!m_PhotonVision.hasBackLeftTarget()){
        xPidOutput = 0;
        yPidOutput = 0;
        rotationPidOutput = 0;
      }
    }else if(m_PhotonVision.hasBackLeftTarget()) {
      backLeftTarget_ID = m_PhotonVision.getBackLeftTargetID();
      if(backLeftTarget_ID == 13 || backLeftTarget_ID == 1) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVision.getRotationMeasurements_BackLeft();
        rotationPidError = m_PhotonVision.getRotationError_Net("BackLeft", "ID13_ID1");
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_Net_BackLeft_ID13_ID1);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Net);
        // Y-PID calculations
        yPidMeasurements = m_PhotonVision.getYMeasurements_BackLeft();
        yPidError = m_PhotonVision.getYError_Net("BackLeft", "ID13_ID1");
        yPidMeasurements = (yPidError > 0.05) ? yPidMeasurements : PhotonConstants.yPidSetPoint_Net_BackLeft_ID13_ID1;
        yPidOutput = -yPidController.calculate(yPidMeasurements, PhotonConstants.yPidSetPoint_Net_BackLeft_ID13_ID1);
        yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Net);
        // X-PID calculations
        xPidMeasurements = m_PhotonVision.getXMeasurements_BackLeft();
        xPidError = m_PhotonVision.getXError_Net("BackLeft", "ID13_ID1");
        xPidMeasurements = (xPidError > 0.05) ? xPidMeasurements : PhotonConstants.xPidSetPoint_Net_BackLeft_ID13_ID1;
        xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_Net_BackLeft_ID13_ID1);
        xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_Net);
      }else if(!m_PhotonVision.hasBackRightTarget()){
          xPidOutput = 0;
          yPidOutput = 0;
          rotationPidOutput = 0;
      }
    }else {
    xPidOutput = 0;
    yPidOutput = 0;
    rotationPidOutput = 0;
    }
  if(m_PhotonVision.isArrive_Net("BackRight", "ID21_ID10") || m_PhotonVision.isArrive_Net("BackRight", "ID20_ID11") || m_PhotonVision.isArrive_Net("BackLeft", "ID13_ID1")) {
      LEDConstants.LEDFlag = true;
      LEDConstants.arrivePosition_Base = true;
      }
  // impl
  if(ElevatorConstants.arriveLevel == 1) {
    xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level1_Net);
    yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level1_Net);
    rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1_Net);
  }else if(ElevatorConstants.arriveLevel == 2) {
    xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level2_Net);
    yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level2_Net);
    rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2_Net);
  }
    m_Swerve.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(0, 0, 0, false);

    if(m_PhotonVision.isArrive_Net("BackRight", "ID21_ID10") || m_PhotonVision.isArrive_Net("BackRight", "ID20_ID11") || m_PhotonVision.isArrive_Net("BackLeft", "ID13_ID1")) {
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
