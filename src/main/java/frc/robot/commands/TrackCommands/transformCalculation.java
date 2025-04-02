// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class transformCalculation extends Command {
  private final PhotonVisionSubsystem m_photonVision;


  // 創建相機相對於基座的變換
  // 注意：WPILib使用米作為單位，所以需要將厘米轉換為米
  private final double x_meters = 0.10;         // 0 cm -> 0 m
  private final double y_meters = 0.35;        // 39 cm -> 0.39 m
  private final double z_meters = 0.50;        // 50 cm -> 0.50 m

  // 將角度轉換為弧度（WPILib使用弧度）
  private final double rx_rad = Math.toRadians(2.0);                     // 0° -> 0 rad
  private final double ry_rad = Math.toRadians(30.0);    // 30° -> π/6 rad
  private final double rz_rad = Math.toRadians(2);                     // 0° -> 0 rad

  private Transform3d tag_T_camera;
  private Transform3d camera_T_robot_base;
  private Transform3d robot_base_T_camera;
  private Transform3d tag_T_robot_base;
    
  public transformCalculation(PhotonVisionSubsystem photonVision) {
    // 創建平移和旋轉
    Translation3d cameraTranslation = new Translation3d(x_meters, y_meters, z_meters);
    Rotation3d cameraRotation = new Rotation3d(rx_rad, ry_rad, rz_rad);

    // 創建相機相對於基座的變換
    camera_T_robot_base = new Transform3d(cameraTranslation, cameraRotation);

    m_photonVision = photonVision;
    addRequirements(m_photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_photonVision.hasFrontLeftTarget()) {
    tag_T_camera = m_photonVision.getFrontLeftTargetPose();
    robot_base_T_camera = camera_T_robot_base.inverse();
    tag_T_robot_base = tag_T_camera.plus(robot_base_T_camera);
    SmartDashboard.putNumber("Test/tag_T_robot_base x", tag_T_robot_base.getTranslation().getX());
    SmartDashboard.putNumber("Test/tag_T_robot_base y", tag_T_robot_base.getTranslation().getY());
    SmartDashboard.putNumber("Test/tag_T_robot_base z", tag_T_robot_base.getTranslation().getZ());
    SmartDashboard.putNumber("Test/tag_T_robot_base rx", tag_T_robot_base.getRotation().getX());
    SmartDashboard.putNumber("Test/tag_T_robot_base ry", tag_T_robot_base.getRotation().getY());
    SmartDashboard.putNumber("Test/tag_T_robot_base rz", tag_T_robot_base.getRotation().getZ());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
