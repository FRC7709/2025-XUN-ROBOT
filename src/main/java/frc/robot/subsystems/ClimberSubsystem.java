// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final SparkMax climbMotor;

  private final SparkMaxConfig climbMotorConfig;

  private final RelativeEncoder motorEncoder;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration absolutedEncoderConfig;

  private final PIDController climbPID;

  private double pidOutput;
  private double goalAngle;
  private boolean climbFlag;

  private String climberState;

  public ClimberSubsystem() {
    goalAngle = ClimberConstants.climbInAngle;
    //Climber Motor
    climbMotor = new SparkMax(ClimberConstants.climbMotor_ID, MotorType.kBrushless);

    climbMotorConfig = new SparkMaxConfig();
    
    climbMotorConfig.idleMode(IdleMode.kBrake);
    climbMotorConfig.inverted(ClimberConstants.firstMotorReverse);

    climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //Climber AbsolutedEncoder
    absolutedEncoder = new CANcoder(ClimberConstants.absolutedEncoder_ID);
    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = ClimberConstants.absolutedEncoderOffset;

    motorEncoder = climbMotor.getEncoder();

    absolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);

    // PID and Feedforward
    climbPID = new PIDController(ClimberConstants.climbPID_Kp, ClimberConstants.climbPID_Ki, ClimberConstants.climbPID_Kd);

    goalAngle = ClimberConstants.primitiveAngle;
    climberState = "Primitive";
    climbFlag = false;
  }

  public double getAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*180;
  }

  public double getAbsolutedPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean arriveSetPoint() {
    return Math.abs(getAngle() - goalAngle) < 2;
  }

  public double getRelativePosition() {
    return motorEncoder.getPosition();
  }

  public void ResetClimber(){
    climberState = "Reset";
    climbFlag = false;
    goalAngle = ClimberConstants.primitiveAngle;
  }

  public void Release(){
    climberState = "Release";
    goalAngle = ClimberConstants.climbOutAngle;
  }

  public void PrepClimb() {
    climberState = "PrepClimb";
    goalAngle = ClimberConstants.climbAngle;
  }

  public void Climb(){
    climberState = "Climb";
    // goalAngle = ClimberConstants.climbInAngle;
  }

  @Override
  public void periodic() {
    if(climberState == "Climb"){
      if(getAngle()<-3 && climbFlag==false){
        climbMotor.setVoltage(0);
        climbFlag = true;
      }
      if(climbFlag==false){
        climbMotor.setVoltage(-8);
      }
    }else{
      // This method will be called once per scheduler run
      pidOutput = climbPID.calculate(getAngle(), goalAngle);
      if(getAngle()<-4) pidOutput = 0;
      // pidOutput = Constants.setMaxOutput(pidOutput, ClimberConstants.climbPIDMaxOutput);
      climbMotor.setVoltage(pidOutput);
    }
    SmartDashboard.putNumber("Climber/AbsolutedPosition", getAbsolutedPosition());
    SmartDashboard.putNumber("Climber/GoalAngle", goalAngle);
    SmartDashboard.putNumber("Climber/CurrentAngle", getAngle());
    SmartDashboard.putNumber("Climber/MotorOutputVolt", pidOutput);
    SmartDashboard.putString("Climber/State", climberState);
    SmartDashboard.putBoolean("Climber/ArrivedSetpoint", arriveSetPoint());
    // SmartDashboard.putNumber("Climber/RelativePositon", getRelativePosition());
  }
}
