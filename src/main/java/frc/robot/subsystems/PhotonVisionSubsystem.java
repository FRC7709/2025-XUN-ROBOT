// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PhotonConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  private final PhotonCamera frontRightCamera;
  private final PhotonCamera frontLeftCamera;

  private PhotonPipelineResult frontRightResult;
  private PhotonPipelineResult frontLeftResult;

  private PhotonTrackedTarget frontRightTarget;
  private PhotonTrackedTarget frontLeftTarget;

  private int frontRightTarget_ID;
  private int frontLeftTarget_ID;

  private double botXMeasurements_FrontRight;
  private double botYMeasurements_FrontRight;
  private double botRotationMeasurements_FrontRight;
  private double botXMeasurements_FrontLeft;
  private double botYMeasurements_FrontLeft;
  private double botRotationMeasurements_FrontLeft;
  private double botXMeasurements_BackRight;
  private double botYMeasurements_BackRight;
  private double botRotationMeasurements_BackRight;
  private double botXMeasurements_BackLeft;
  private double botYMeasurements_BackLeft;
  private double botRotationMeasurements_BackLeft;


  public PhotonVisionSubsystem() {
    frontRightCamera = new PhotonCamera("OV9281_FrontRight");
    frontLeftCamera = new PhotonCamera("OV9281_FrontLeft");
  }

  public int getFrontRightTargetID() {
    return frontRightTarget_ID;
  }

  public int getFrontLeftTargetID() {
    return frontLeftTarget_ID;
  }

  public boolean hasFrontRightTarget() {
    return frontRightResult.hasTargets();
  }

  public boolean hasFrontLeftTarget() {
    return frontLeftResult.hasTargets();
  }


  public boolean hasFrontTarget() {
    if(hasFrontRightTarget() || hasFrontLeftTarget()) return true;
    return false;
  }

  public boolean hasTarget() {
    if(hasFrontTarget()) return true;
    return false;
  }

  public Transform3d getFrontRightTargetPose() {
    return frontRightTarget.getBestCameraToTarget();
  }

  public Transform3d getFrontLeftTargetPose() {
    return frontLeftTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_FrontRight() {
    // return frontRightTarget.getBestCameraToTarget().plus(frontRightToRobot);
    return frontRightTarget.getBestCameraToTarget();
  }

  public Transform3d getRobotToTargetPose_FrontLeft() {
    return frontLeftTarget.getBestCameraToTarget();
  }

  // public Optional<Matrix<N3, N3>> getCameraMatrix(String camera) {
  //   if(camera == "FrontRight") return frontRightCamera.getCameraMatrix();
  //   if(camera == "FrontLeft") return frontLeftCamera.getCameraMatrix();
  //   if(camera == "backRight") return backRightCamera.getCameraMatrix();
  //   if(camera == "backLeft") return backLeftCamera.getCameraMatrix();
  //   return null;
  // }

  // public Optional<Matrix<N8, N1>> getCameraDistCoeffs(String camera) {
  //   if(camera == "FrontRight") return frontRightCamera.getDistCoeffs();
  //   if(camera == "FrontLeft") return frontLeftCamera.getDistCoeffs();
  //   if(camera == "backRight") return backRightCamera.getDistCoeffs();
  //   if(camera == "backLeft") return backLeftCamera.getDistCoeffs();
  //   return null;
  // }

  public double getXMeasurements_FrontRight() {
    return botXMeasurements_FrontRight;
  }

  public double getYMeasurements_FrontRight() {
    return botYMeasurements_FrontRight;
  }

  public double getRotationMeasurements_FrontRight() {
    return botRotationMeasurements_FrontRight;
  }

  public double getXMeasurements_FrontLeft() {
    return botXMeasurements_FrontLeft;
  }

  public double getYMeasurements_FrontLeft() {
    return botYMeasurements_FrontLeft;
  }

  public double getRotationMeasurements_FrontLeft() {
    return botRotationMeasurements_FrontLeft;
  }

  public double getXMeasurements_BackRight() {
    return botXMeasurements_BackRight;
  }

  public double getYMeasurements_BackRight() {
    return botYMeasurements_BackRight;
  }

  public double getRotationMeasurements_BackRight() {
    return botRotationMeasurements_BackRight;
  }

  public double getXMeasurements_BackLeft() {
    return botXMeasurements_BackLeft;
  }

  public double getYMeasurements_BackLeft() {
    return botYMeasurements_BackLeft;
  }

  public double getRotationMeasurements_BackLeft() {
    return botRotationMeasurements_BackLeft;
  }


  public double getRotationError_Net(String camera, String ID) {
    if(camera == "BackRight") {
      if(ID == "ID20_ID11") return Math.abs(getYMeasurements_BackRight() - PhotonConstants.yPidSetPoint_Net_BackRight_ID20_ID11);
      else return Math.abs(getYMeasurements_BackRight() - PhotonConstants.yPidSetPoint_Net_BackRight_ID21_ID10);
    }else {
      return Math.abs(getYMeasurements_BackLeft() - PhotonConstants.yPidSetPoint_Net_BackLeft_ID13_ID1);
    }
  }

  public double getXError_Processor() {
    return Math.abs(getXMeasurements_BackRight() - PhotonConstants.xPidSetPoint_Processor_BackRight);
  }

  public double getYError_Processor() {
    return Math.abs(getYMeasurements_BackRight() - PhotonConstants.yPidSetPoint_Processor_BackRight);
  }

  public double getRotationError_Processor() {
    return Math.abs(getRotationMeasurements_BackRight() - PhotonConstants.rotationPidSetPoint_Processor_BackRight);
  }
  
  
  public double getXError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getXMeasurements_FrontLeft() - PhotonConstants.xPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getXMeasurements_FrontRight() - PhotonConstants.xPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getXMeasurements_FrontRight() - PhotonConstants.xPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getXMeasurements_FrontLeft() - PhotonConstants.xPidSetPoint_MiddleReef_FrontLeft);
  }

  public double getYError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getYMeasurements_FrontLeft() - PhotonConstants.yPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getYMeasurements_FrontRight() - PhotonConstants.yPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getYMeasurements_FrontRight() - PhotonConstants.yPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getYMeasurements_FrontLeft() - PhotonConstants.yPidSetPoint_MiddleReef_FrontLeft);
  }

  public double getRotationError_Reef(String reef) {
    if(reef == "RightReef") return Math.abs(getRotationMeasurements_FrontLeft() - PhotonConstants.rotationPidSetPoint_RightReef);
    else if(reef == "LeftReef") return Math.abs(getRotationMeasurements_FrontRight() - PhotonConstants.rotationPidSetPoint_LeftReef);
    else if(reef == "MiddleReef_FrontRight") return Math.abs(getRotationMeasurements_FrontRight() - PhotonConstants.rotationPidSetPoint_MiddleReef_FrontRight);
    else return Math.abs(getRotationMeasurements_FrontLeft() - PhotonConstants.rotationPidSetPoint_MiddleReef_FrontLeft);
  }



  public boolean isArrive_Reef(String reef) {
    double x_range = 0.04;
    double y_range = 0.03;
    double rotation_range = 0.5;
    if(reef == "RightReef") {
      if((getXError_Reef("RightReef")) <= x_range && (getYError_Reef("RightReef") <= y_range) && (getRotationError_Reef("RightReef") <= rotation_range) && hasFrontLeftTarget()) return true;
      else return false;
    }else if(reef == "LeftReef") {
      if((getXError_Reef("LeftReef")) <= x_range && (getYError_Reef("LeftReef") <= y_range) && (getRotationError_Reef("LeftReef") <= rotation_range) && hasFrontRightTarget()) return true;
      else return false;
    }else if(reef == "MiddleReef_FrontRight") {
      if((getXError_Reef("MiddleReef_FrontRight") <= x_range) && (getYError_Reef("MiddleReef_FrontRight") <= y_range) && (getRotationError_Reef("MiddleReef_FrontRight") <= rotation_range)) return true;
      else return false;
    }else {
      if((getXError_Reef("MiddleReef_FrontLeft") <= x_range) && (getYError_Reef("MiddleReef_FrontLeft") <= y_range) && (getRotationError_Reef("MiddleReef_FrontLeft") <= rotation_range)) return true;
      else return false;
    } 
  }


  public boolean isArrive_Processor() {
    if(getXError_Processor() <= 0.03 && getYError_Processor() <= 0.03 && getRotationError_Processor() <= 0.5) return true;
    else return false;
  }

  public PhotonPipelineResult getResult(String camera) {
    if(camera == "FrontRight") return frontRightResult;
    if(camera == "FrontLeft") return frontLeftResult;
    return null;
  }

  // public static Translation3d rotateTranslation(Translation3d translation, double yaw) {
  //   double cosYaw = Math.cos(yaw);
  //   double sinYaw = Math.sin(yaw);

  //   // 旋轉 X, Y 坐標
  //   double newX = translation.getX() * cosYaw - translation.getY() * sinYaw;
  //   double newY = translation.getX() * sinYaw + translation.getY() * cosYaw;
  //   double newZ = translation.getZ(); // Z 不受影響

  //   return new Translation3d(newX, newY, newZ);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontRightResult = frontRightCamera.getLatestResult();
    frontRightTarget = frontRightResult.getBestTarget();
    // frontRightTargets = frontRightResult.getTargets();
    frontLeftResult = frontLeftCamera.getLatestResult();
    frontLeftTarget = frontLeftResult.getBestTarget();

    SmartDashboard.putBoolean("Photon/FR_hasTarget", hasFrontRightTarget());
    SmartDashboard.putBoolean("Photon/FL_hasTarget", hasFrontLeftTarget());


    // backRightTargets = backRightResult.getTargets();
    if(hasFrontRightTarget()) {
      botXMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getTranslation().getX();
      botYMeasurements_FrontRight = getRobotToTargetPose_FrontRight().getTranslation().getY();
      botRotationMeasurements_FrontRight = Math.toDegrees(getRobotToTargetPose_FrontRight().getRotation().getAngle());

      // Translation3d botTranslation_FrontRight = getRobotToTargetPose_FrontRight().getTranslation();
      // Rotation3d botRotation_FrontRight = getRobotToTargetPose_FrontRight().getRotation();
      // Pose3d CameraToTag_FrontRight = new Pose3d(botTranslation_FrontRight, botRotation_FrontRight);
      // Translation3d robotToFrontRight_Translation3d = robotToFrontRight.getTranslation();
      // Rotation3d robotToFrontRight_Rotation3d = robotToFrontRight.getRotation();
      // Pose3d robotToFrontRightPose = new Pose3d(robotToFrontRight_Translation3d, robotToFrontRight_Rotation3d);

      // int frontRightTag_ID = frontRightTarget.getFiducialId();

      // Pose3d tagPose = aprilTagFieldLayout.getTagPose(frontRightTag_ID).get();

      // Pose3d robotPose_FrontRight = robotToFrontRightPose.transformBy(getFrontRightTargetPose());

      // botXMeasurements_FrontRight = robotPose_FrontRight.getTranslation().getX();
      // botYMeasurements_FrontRight = robotPose_FrontRight.getTranslation().getY();
      // botRotationMeasurements_FrontRight = Math.toDegrees(robotPose_FrontRight.getRotation().getAngle());


      // Translation3d rotatedTranslation = rotateTranslation(getRobotToTargetPose_FrontRight().getTranslation(), robotToFrontRight.getRotation().getZ());
      // Translation3d robotToTagTranslation = rotatedTranslation.plus(robotToFrontRight.getTranslation());
      // Rotation3d robotToTagRotation = new Rotation3d(
      // getRobotToTargetPose_FrontRight().getRotation().getX() + robotToFrontRight.getRotation().getX(),
      // getRobotToTargetPose_FrontRight().getRotation().getY() + robotToFrontRight.getRotation().getY(),
      // getRobotToTargetPose_FrontRight().getRotation().getZ() + robotToFrontRight.getRotation().getZ()
      // );

      // botXMeasurements_FrontRight = robotToTagTranslation.getX();
      // botYMeasurements_FrontRight = robotToTagTranslation.getY();
      // botRotationMeasurements_FrontRight = Math.toDegrees(robotToTagRotation.getAngle());
      
      frontRightTarget_ID = frontRightTarget.getFiducialId();
      
      SmartDashboard.putNumber("Photon/FR_TargetID", frontRightTarget_ID);
      SmartDashboard.putNumber("Photon/FR_xMeasurement", botXMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FR_yMeasurement", botYMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FR_rotationMeasurement", botRotationMeasurements_FrontRight);
      SmartDashboard.putNumber("Photon/FR_xError", getXError_Reef("LeftReef"));
      SmartDashboard.putNumber("Photon/FR_yError", getYError_Reef("LeftReef"));
      SmartDashboard.putNumber("Photon/FR_rotationError", getRotationError_Reef("LeftReef"));

    }else {
      botXMeasurements_FrontRight = 0;
      botYMeasurements_FrontRight = 0;
      botRotationMeasurements_FrontRight = 0;
      frontRightTarget_ID = 0;
    }
    // If the camera has a target, get the target pose
    if(hasFrontLeftTarget()) {
      botXMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getX();
      botYMeasurements_FrontLeft = getRobotToTargetPose_FrontLeft().getY();
      botRotationMeasurements_FrontLeft = Math.toDegrees(getRobotToTargetPose_FrontLeft().getRotation().getAngle());

      frontLeftTarget_ID = frontLeftTarget.getFiducialId();
      
      SmartDashboard.putNumber("Photon/FL_TargetID", frontLeftTarget_ID);
      SmartDashboard.putNumber("Photon/FL_xMeasurement", botXMeasurements_FrontLeft);
      SmartDashboard.putNumber("Photon/FL_yMeasurement", botYMeasurements_FrontLeft);
      SmartDashboard.putNumber("Photon/FL_rotationMeasurement", botRotationMeasurements_FrontLeft);
      SmartDashboard.putNumber("Photon/FL_xError", getXError_Reef("RightReef"));
      SmartDashboard.putNumber("Photon/FL_yError", getYError_Reef("RightReef"));
      SmartDashboard.putNumber("Photon/FL_rotationError", getRotationError_Reef("RightReef"));
    }else {
      botXMeasurements_FrontLeft = 0;
      botYMeasurements_FrontLeft = 0;
      botRotationMeasurements_FrontLeft = 0;
      frontLeftTarget_ID = 0;
    }

  
    if(LEDConstants.hasGamePiece && hasFrontRightTarget()) {
      LEDConstants.canTrackLeft = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.canTrackLeft = false;
      LEDConstants.LEDFlag = true;
    }
    if(LEDConstants.hasGamePiece && hasFrontLeftTarget()) {
      LEDConstants.canTrackRight = true;
      LEDConstants.LEDFlag = true;
    }else {
      LEDConstants.canTrackRight = false;
      LEDConstants.LEDFlag = true;
    }   
  }
}
