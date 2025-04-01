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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackLeftReef extends Command {
  /** Creates a new TrackReef. */
  private final PhotonVisionSubsystem m_PhotonVision;
  private final SwerveSubsystem_Kraken m_Swerve;

  private PIDController rotationPidController;
  private PIDController xPidController;
  private PIDController yPidController;

  private double xPidMeasurements;
  private double yMeasurement;
  private double rotationMeasurement;

  private double xPidError;
  private double yPidError;
  private double rotationPidError;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationPidOutput;

  public TrackLeftReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem_Kraken swerveSubsystem) {
    this.m_PhotonVision = photonVisionSubsystem;
    this.m_Swerve = swerveSubsystem;

    addRequirements(m_PhotonVision, m_Swerve);
    // PID
    xPidController = new PIDController(PhotonConstants.xPidController_Kp, PhotonConstants.xPidController_Ki, PhotonConstants.xPidController_Kd);
    yPidController = new PIDController(PhotonConstants.yPidController_Kp, PhotonConstants.yPidController_Ki, PhotonConstants.yPidController_Kd);
    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);
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
      rotationPidError = m_PhotonVision.getRotationError_Reef("LeftReef");
      rotationMeasurement = (rotationPidError > 0.5) ? rotationMeasurement : PhotonConstants.rotationPidSetPoint_LeftReef;
      rotationPidOutput = rotationPidController.calculate(rotationMeasurement, PhotonConstants.rotationPidSetPoint_LeftReef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_Reef);
      // Y-PID calculations
      yMeasurement = m_PhotonVision.getYMeasurements_FrontRight();
      yPidError = m_PhotonVision.getYError_Reef("LeftReef");
      yMeasurement = (yPidError > 0.02) ? yMeasurement : PhotonConstants.yPidSetPoint_LeftReef;
      yPidOutput = -yPidController.calculate(yMeasurement, PhotonConstants.yPidSetPoint_LeftReef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_Reef);
      // X-PID calculations
      xPidMeasurements = m_PhotonVision.getXMeasurements_FrontRight();
      xPidError = m_PhotonVision.getRotationError_Reef("LeftReef");
      xPidMeasurements = (xPidError > 0.02) ? xPidMeasurements : PhotonConstants.xPidSetPoint_LeftReef;
      xPidOutput = -xPidController.calculate(xPidMeasurements, PhotonConstants.xPidSetPoint_LeftReef);
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidSetPoint_LeftReef);
      // Control led
      if(m_PhotonVision.isArrive_Reef("LeftReef")) {
        LEDConstants.arrivePosition_Base = true;
        LEDConstants.LEDFlag = true;
      }
    }else {
      xPidOutput = 0;
      yPidOutput = 0;
      rotationPidOutput = 0;
    }
    // LED control 
    // Speed limit protection
    if(ElevatorConstants.arriveLevel == 1) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level1_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level1_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1_Reef);
    }else if(ElevatorConstants.arriveLevel == 2) {
      xPidOutput = Constants.setMaxOutput(xPidOutput, PhotonConstants.xPidMaxOutput_NeedSlow_Level2_Reef);
      yPidOutput = Constants.setMaxOutput(yPidOutput, PhotonConstants.yPidMaxOutput_NeedSlow_Level2_Reef);
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2_Reef);
    }
    SmartDashboard.putNumber("Photon/RotationPidOutput", rotationPidOutput);
    // Impl
    m_Swerve.drive(xPidOutput, yPidOutput, rotationPidOutput, false);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(0, 0, 0, false);

    if(m_PhotonVision.isArrive_Reef("LeftReef")) {
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
