// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

public class ManualDrive_TrackLeftCoralStation extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem_Kraken m_SwerveSubsystem_Kraken;
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier ySpeedFunc;
  private final BooleanSupplier isSlowFunc;
  // private final BooleanSupplier needSlowFunc;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;

  private final PIDController rotationPidController;

  private int backLeftTarget_ID;
  private double rotationPidMeasurements;
  private double rotationPidError;
  private double rotationPidOutput;
  private double xSpeed;
  private double ySpeed;
  private boolean isSlow;
  // private boolean needSlow;
  public ManualDrive_TrackLeftCoralStation(SwerveSubsystem_Kraken swerveSubsystem, PhotonVisionSubsystem photonVisionSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem_Kraken = swerveSubsystem;
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.isSlowFunc = isSlow;
    // this.needSlowFunc = needSlow;

    rotationPidController = new PIDController(PhotonConstants.rotationPidController_Kp, PhotonConstants.rotationPidController_Ki, PhotonConstants.rotationPidController_Kd);

    this.xLimiter = new SlewRateLimiter(5.5);
    this.yLimiter = new SlewRateLimiter(5.5);

    

    addRequirements(m_SwerveSubsystem_Kraken);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.xSpeed = -xSpeedFunc.getAsDouble();
    this.ySpeed = -ySpeedFunc.getAsDouble();

    this.xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
    this.ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);

    this.xSpeed = xLimiter.calculate(this.xSpeed);
    this.ySpeed = yLimiter.calculate(this.ySpeed);

    this.isSlow = isSlowFunc.getAsBoolean();
    // this.needSlow = needSlowFunc.getAsBoolean();

    if(m_PhotonVisionSubsystem.hasBackLeftTarget()) {
      backLeftTarget_ID = m_PhotonVisionSubsystem.getBackLeftTargetID();
      if(backLeftTarget_ID == 13 || backLeftTarget_ID == 1) {
        // Rotation-PID calculations
        rotationPidMeasurements = m_PhotonVisionSubsystem.getRotationMeasurements_BackLeft();
        rotationPidError = m_PhotonVisionSubsystem.getRotationError_CoralStation("BackLeft", "LeftCoralStation");
        rotationPidMeasurements = (rotationPidError > 0.5) ? rotationPidMeasurements : PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackLeft;
        rotationPidOutput = rotationPidController.calculate(rotationPidMeasurements, PhotonConstants.rotationPidSetPoint_LeftCoralStation_BackLeft);
        rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_CoralStation);
      }else {
        rotationPidOutput = 0;
      }
    }else {;
      rotationPidOutput = 0;
    }

    if((isSlow && ElevatorConstants.arriveLevel == 2) || ElevatorConstants.arriveLevel == 2) {
      xSpeed = xSpeed*Math.abs(xSpeed)*0.1;
      ySpeed = ySpeed*Math.abs(ySpeed)*0.1;
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level2);
    }else if(ElevatorConstants.arriveLevel == 1 || isSlow) {
      xSpeed = xSpeed*Math.abs(xSpeed)*0.2;
      ySpeed = ySpeed*Math.abs(ySpeed)*0.2;
      rotationPidOutput = Constants.setMaxOutput(rotationPidOutput, PhotonConstants.rotationPidMaxOutput_NeedSlow_Level1);
    }else {
      xSpeed = xSpeed*Math.abs(xSpeed)*0.8;
      ySpeed = ySpeed*Math.abs(ySpeed)*0.8;
    }

    SmartDashboard.putNumber("ManualDrive/Xspeed", xSpeed);
    SmartDashboard.putNumber("ManualDrive/Yspeed", ySpeed);
    SmartDashboard.putNumber("ManualDrive/Zspeed", rotationPidOutput);

    m_SwerveSubsystem_Kraken.drive(this.xSpeed, this.ySpeed, rotationPidOutput, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem_Kraken.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
