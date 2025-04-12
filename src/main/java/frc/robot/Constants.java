// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }

  public static class Module_KrakenConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2RadPerSec = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;
    public static final double driveEncoderRot2RadPerMin = turningEncoderRot2RadPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0.00001;

    public static final double drivePidController_Kp = 0;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2;
    public static final double driveFeedforward_Ka = 0;

    public static final double driveFeedforward_Auto_Ks = 0.13;
    public static final double driveFeedforward_Auto_Kv = 1.73;
    public static final double driveFeedforward_Auto_Ka = 1.41;

  }

  public class Swerve_KrakenConstants {
    public static final int leftFrontDrive_ID = 2;
    public static final int leftBackDrive_ID = 1;
    public static final int rightFrontDrive_ID = 3;
    public static final int rightBackDrive_ID = 4;

    public static final int leftFrontTurning_ID = 6;
    public static final int leftBackTurning_ID = 5;
    public static final int rightFrontTurning_ID = 7;
    public static final int rightBackTurning_ID = 8;

    public static final int leftFrontAbsolutedEncoder_ID = 42;
    public static final int leftBackAbsolutedEncoder_ID = 41;
    public static final int rightFrontAbsolutedEncoder_ID = 43;
    public static final int rightBackAbsolutedEncoder_ID = 44;

    public static final double leftFrontOffset = -0.352806;
    public static final double leftBackOffset = 0.138427;
    public static final double rightFrontOffset = -0.280517;
    public static final double rightBackOffset = 0.161621;
    public static final int gyro_ID = 56;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;


    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2), 
      new Translation2d(kModuleDistance/2, -kModuleDistance/2), 
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double pathingMoving_Kp = 5;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0.6;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 850;
  
  }

  public class PhotonConstants {
    public static final double xPid_Kp = 0.6;
    public static final double xPid_Ki = 0;
    public static final double xPid_Kd = 0;

    public static final double yPid_Kp = 0.6;
    public static final double yPid_Ki = 0;
    public static final double yPid_Kd = 0.0015;

    public static final double rotationPid_Kp = 0.0019;
    public static final double rotationPid_Ki = 0;
    public static final double rotationPid_Kd = 0.0001;

    public static final double xPidMaxOutput = 0.4;
    public static final double yPidMaxOutput = 0.4;
    public static final double rotationPidMaxOutput = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Level1 = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1 = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1 = 0.2;
    public static final double xPidMaxOutput_NeedSlow_Level2 = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level2 = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level2 = 0.2;

    public static final double xPidMaxOutput_Reef = 0.3;
    public static final double yPidMaxOutput_Reef = 0.3;
    public static final double rotationPidMaxOutput_Reef = 0.2;
    public static final double xPidMaxOutput_NeedSlow_Level1_Reef= 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Reef = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Reef = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Reef= 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Reef = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Reef = 0.05;

    public static final double xPidMaxOutput_CoralStation = 0.4;
    public static final double yPidMaxOutput_CoralStation = 0.4;
    public static final double rotationPidMaxOutput_CoralStation = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Level1_CoralStation = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_CoralStation = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_CoralStation = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_CoralStation = 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_CoralStation = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_CoralStation = 0.05;

    public static final double xPidMaxOutput_Cage = 0.4;
    public static final double yPidMaxOutput_Cage = 0.4;
    public static final double rotationPidMaxOutput_Cage = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Level1_Cage = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Cage = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Cage = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Cage = 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Cage = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Cage = 0.05;

    public static final double xPidMaxOutput_Reef_Auto = 0.3;
    public static final double yPidMaxOutput_Reef_Auto = 0.3;
    public static final double rotationPidMaxOutput_Reef_Auto = 0.2;
    public static final double xPidMaxOutput_NeedSlow_Level1_Reef_Auto= 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Reef_Auto = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Reef_Auto = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Reef_Auto= 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Reef_Auto = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Reef_Auto = 0.05;

    public static final double xPidMaxOutput_Net_Auto = 0.4;
    public static final double yPidMaxOutput_Net_Auto = 0.4;
    public static final double rotationPidMaxOutput_Net_Auto = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Level1_Net_Auto = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Net_Auto = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Net_Auto = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Net_Auto = 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Net_Auto = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Net_Auto = 0.05;

    public static final double xPidMaxOutput_Processor = 0.4;
    public static final double yPidMaxOutput_Processor = 0.4;
    public static final double rotationPidMaxOutput_Processor = 0.2;
    public static final double xPidMaxOutput_NeedSlow_Level1_Processor = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Processor = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Processor = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Processor = 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Processor = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Processor = 0.05;

    public static final double xPidMaxOutput_Net = 0.4;
    public static final double yPidMaxOutput_Net = 0.4;
    public static final double rotationPidMaxOutput_Net = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Level1_Net = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Level1_Net = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Level1_Net = 0.1;
    public static final double xPidMaxOutput_NeedSlow_Level2_Net = 0.1;
    public static final double yPidMaxOutput_NeedSlow_Level2_Net = 0.1;
    public static final double rotationPidMaxOutput_NeedSlow_Level2_Net = 0.05;

    public static final double xPidSetPoint_RightReef = 0.402; 
    public static final double yPidSetPoint_RightReef = -0.145;
    public static final double rotationPidSetPoint_RightReef = 180.9;

    public static final double xPidSetPoint_LeftReef = 0.4057;
    public static final double yPidSetPoint_LeftReef = 0.1472;
    public static final double rotationPidSetPoint_LeftReef = 178.5;

    public static final double xPidSetPoint_MiddleReef_FrontRight = 0.411;
    public static final double yPidSetPoint_MiddleReef_FrontRight = 0.172;
    public static final double rotationPidSetPoint_MiddleReef_FrontRight = 178.7;

    public static final double xPidSetPoint_MiddleReef_FrontLeft = 0.395;
    public static final double yPidSetPoint_MiddleReef_FrontLeft = -0.257;
    public static final double rotationPidSetPoint_MiddleReef_FrontLeft = 182.8;

    public static final double xPidSetPoint_LeftCoralStation_BackLeft = 0;
    public static final double yPidSetPoint_LeftCoralStation_BackLeft = 0;
    public static final double rotationPidSetPoint_LeftCoralStation_BackLeft = 0;

    public static final double xPidSetPoint_RightCoralStation_BackLeft = 0;
    public static final double yPidSetPoint_RightCoralStation_BackLeft = 0;
    public static final double rotationPidSetPoint_RightCoralStation_BackLeft = 0;

    public static final double xPidSetPoint_LeftCoralStation_BackRight = 0;
    public static final double yPidSetPoint_LeftCoralStation_BackRight = 0;
    public static final double rotationPidSetPoint_LeftCoralStation_BackRight = 0;

    public static final double xPidSetPoint_RightCoralStation_BackRight = 0;
    public static final double yPidSetPoint_RightCoralStation_BackRight = 0;
    public static final double rotationPidSetPoint_RightCoralStation_BackRight = 0;

    public static final double xPidSetPoint_Cage_FrontRight = 0;
    public static final double yPidSetPoint_Cage_FrontRight = 0;
    public static final double rotationPidSetPoint_Cage_FrontRight = 0;

    public static final double xPidSetPoint_Cage_FrontLeft = 0;
    public static final double yPidSetPoint_Cage_FrontLeft = 0;
    public static final double rotationPidSetPoint_Cage_FrontLeft = 0;

    public static final double xPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double yPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID20_ID11 = 0;

    public static final double xPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double yPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID21_ID10 = 0;

    public static final double xPidSetPoint_Processor_FrontRight = 0;
    public static final double yPidSetPoint_Processor_FrontRight = 0;
    public static final double rotationPidSetPoint_Processor_FrontRight = 0;

    public static final double xPidSetPoint_Processor_FrontLeft = 0;
    public static final double yPidSetPoint_Processor_FrontLeft = 0;
    public static final double rotationPidSetPoint_Processor_FrontLeft = 0;

    public static final double xPidSetPoint_Processor_BackRight = 0;
    public static final double yPidSetPoint_Processor_BackRight = 0;
    public static final double rotationPidSetPoint_Processor_BackRight = 0;

    public static final double xPidSetPoint_Net_BackRight_ID20_ID11 = 0;
    public static final double yPidSetPoint_Net_BackRight_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Net_BackRight_ID20_ID11 = 0;

    public static final double xPidSetPoint_Net_BackRight_ID21_ID10 = 0;
    public static final double yPidSetPoint_Net_BackRight_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Net_BackRight_ID21_ID10 = 0;

    public static final double xPidSetPoint_Net_BackLeft_ID13_ID1 = 0;
    public static final double yPidSetPoint_Net_BackLeft_ID13_ID1 = 0;
    public static final double rotationPidSetPoint_Net_BackLeft_ID13_ID1 = 0;

    public static final double arriveXPosition_Reef = 0;
    public static final double arriveXPosition_Cage = 0;
    public static final double arrivePosition_Net = 0;

    public static final double tooClosePosition_Reef = 0;
    public static final double tooClosePosition_Cage = 0;
    public static final double tooClosePosition_Net = 0;

  }

  public class ElevatorConstants {
    public static final int elevator_FirstMotor_ID = 11;
    public static final int elevator_SecondMotor_ID = 12;

    public static final double prepareForScorePosition_Net = 20;
    public static final double prepareForScorePosition_Coral = 20;
    public static final double primitivePosition = -0.2;//-0.14
    public static final double coralL1Position = 4;//3.72
    public static final double coralL2Position = 11.44;//7.42
    public static final double coralL3Position = 22.5;//21.42
    public static final double coralL4Position = 43.34;//42.52
    public static final double coralStationPosition = -0.2;//0.12

    public static final double algaeFloorPosition = -0.2;//0.12
    public static final double algaeNetPosition = 45.7;//0.12
    public static final double algaeL2Position = 2.84;//3.82
    public static final double algaeL3Position = 16;//15.82

    public static final double algaeProccesorPosition = -0.2;//0.12

    public static int arriveLevel = 0;
  }

  public static class WristConstants{
    // Motor
    public static final int wristMotor_ID = 14;
    // Absoluted Encoder
    public static final int CANcoder_ID = 45;
    public static final double encoderOffset = 0.140625;
    // PID
    public static final double Kp = 0.075;//0.0048  
    public static final double Ki = 0.001;
    public static final double Kd = 0.005;//0.0001
    public static final double PIDMaxOutput = 2;
    // Feedforward
    public static final double Kg = 0.3;//
    // Setpoint
    public static final double primitiveAngle = 82;
    public static final double primitiveAngle_HasCoral = 82;
    public static final double coralL1Angle = 82;
    public static final double coralL2Angle = 75;
    public static final double coralL3Angle = 75;
    public static final double coralL4Angle = 56;//not yet
    public static final double coralStationAngle = 75;
    public static final double coralL4UpAngle = 70;
    public static final double algaeFloorAngle = 7;
    public static final double algaeNetAngle = 100;//not yet
    public static final double netUpAngle = 82;
    public static final double algaeLowInAngle = 64;
    public static final double algaeHighInAngle = 64;
    public static final double algaeProccesorAngle = 9;//not yet
    public static final double algaeRemoveAngle = 75;
  }

  public static class EndEffectorConstants {
    // Motor
    public static final int wheelMotor_ID = 13;
    // IR sensor 
    public static final int irSensor_CoralFirst_ID = 0;
    public static final int irSensor_CoralSecond_ID = 1;
    public static final int irSensor_Algae_ID = 2;   
    // Output voltage
    public static final double coralL1OutVol = -3.5;
    public static final double coralL2OutVol = -2;
    public static final double coralL3OutVol = -2;
    public static final double coralL4OutVol = -2;
    public static final double coralTurnMore = -0.5;
    public static final double coralInSpeed_RotionPerSecond = -32;
    public static final double coralInSpeedSlow_RotationPerSecond = -5;
    public static final double algaeFloorInVol = -6;
    public static final double algaeShootNetVol = 6;
    public static final double algaeLowInVol = -6;
    public static final double algaeHighInVol = -6;
    public static final double algaeShootProcessorVol = 3;
    public static final double algaeHoldVol = -4;  
    public static final double algaeOutVol = 6; 
    public static final double algaeRemoveVol = -6;
  }

  public static class ClimberConstants { 
    public static final int climbMotor_ID = 23;

    public static final int absolutedEncoder_ID = 40;

    public static final boolean firstMotorReverse = false;

    public static final double absolutedEncoderOffset = -0.31762;

    public static final double climbPID_Kp = 0.8;
    public static final double climbPID_Ki = 0;
    public static final double climbPID_Kd = 0;

    public static final double climbPIDMinRange = 0;
    public static final double climbPIDMaxRange = 0;

    public static final double climbPIDMinOutput = 0;
    public static final double climbPIDMaxOutput = 0.4;

    public static final double primitiveAngle = 16;
    public static final double climbOutAngle = 67.5;
    public static final double climbAngle = 29; 
    public static final double climbInAngle = -3.2;
  }

  public class LEDConstants {
    public static final int candle_ID = 46;

    public static final int ledNum = 20;

    public static boolean LEDFlag = false;
    public static boolean hasGamePiece = false;
    public static boolean hasAlgae = false;
    public static boolean intakeGamePiece = false;
    public static boolean tracking = false;
    public static boolean arrivePosition_Intake = false;
    public static boolean hasFrontRightTarget = false;
    public static boolean hasFrontLeftTarget = false;
    public static boolean canTrackLeft = false;
    public static boolean canTrackRight = false;
    public static boolean canTrackMiddle = false;
    public static boolean noTarget = false;
    public static boolean intakeArriving = false;
    public static boolean arrivePosition_Base = false;
    public static boolean shootGamePiece = false;
    public static boolean onCage = false;
    public static boolean climbing = false;
    public static boolean fireAnimation = false;
    public static boolean normal = false;
  }

  public static class Module_NeoConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final double drivePidController_Kp = 0.2;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2.58;

  }

  public class Swerve_NeoConstants {
    public static final int rightFrontDrive_ID = 8;
    public static final int rightBackDrive_ID = 13;
    public static final int leftFrontDrive_ID = 1;
    public static final int leftBackDrive_ID = 19;

    public static final int rightFrontTurning_ID = 16;
    public static final int rightBackTurning_ID = 26;
    public static final int leftFrontTurning_ID = 7;
    public static final int leftBackTurning_ID = 29;

    public static final int rightFrontAbsolutedEncoder_ID = 41;
    public static final int rightBackAbsolutedEncoder_ID = 42;
    public static final int leftFrontAbsolutedEncoder_ID = 43;
    public static final int leftBackAbsolutedEncoder_ID = 44;

    public static final double leftFrontOffset = -0.106689;
    public static final double leftBackOffset = 0.380371;
    public static final double rightFrontOffset = -0.058837;
    public static final double rightBackOffset = 0.416503;

    public static final int gyro_ID = 55;

    public static final double kModuleDistance = 22.24*0.0254;
    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2));

    
    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 4.6;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 720;

  }
}
