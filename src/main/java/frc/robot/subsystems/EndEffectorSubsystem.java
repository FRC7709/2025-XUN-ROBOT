// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.WristConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX wheelMotor;
  private final TalonFX pivotMotor;
  private final CANcoder pivotCANcoder;
  private final DigitalInput irSensor_CoralFirst;
  private final DigitalInput irSensor_CoralSecond;
  private final DigitalInput irSensor_Algae;

  private final TalonFXConfiguration wheelConfig;
  private final TalonFXConfiguration pivotConfig;
  private final CANcoderConfiguration CANcoderConfig;
  private final MotionMagicConfigs wheelMotionMagicConfig;
  private final MotionMagicConfigs turnMotionMagicConfigs;
  private final Slot0Configs wheelMotor_Slot0;
  private final MotionMagicVelocityVoltage requst_wheelSpeed;
  

  private final PIDController armPID;
  private ArmFeedforward armFeedforward;

  private double pidOutput;
  private double feedforwardOutput;
  private double output;
  private double goalAngle;

  private Debouncer m_Debouncer_first;
  private Debouncer m_Debouncer_second;

  private String pivotState = "IDLE"; // State of the end effector subsystem
  private String wheelState = "STOP"; // State of the motor (wheel or pivot)

    public EndEffectorSubsystem() {
      // Motor controller
      wheelMotor = new TalonFX(EndEffectorConstants.wheelMotor_ID);
      pivotMotor = new TalonFX(WristConstants.wristMotor_ID);
      // Encoder
      pivotCANcoder = new CANcoder(WristConstants.CANcoder_ID);
      // IR sensor
      irSensor_CoralFirst = new DigitalInput(EndEffectorConstants.irSensor_CoralFirst_ID);
      irSensor_CoralSecond = new DigitalInput(EndEffectorConstants.irSensor_CoralSecond_ID);
      irSensor_Algae = new DigitalInput(EndEffectorConstants.irSensor_Algae_ID);
      // Init goal angle
      goalAngle = WristConstants.primitiveAngle;
  
      // Motor Configurations
      wheelConfig = new TalonFXConfiguration();
      pivotConfig = new TalonFXConfiguration();
      wheelMotionMagicConfig = new MotionMagicConfigs();
      turnMotionMagicConfigs = new MotionMagicConfigs();
      wheelMotor_Slot0 = wheelConfig.Slot0;
      requst_wheelSpeed = new MotionMagicVelocityVoltage(0);
  
      wheelConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      
      //slot0
      wheelMotor_Slot0.kS = 0;
      wheelMotor_Slot0.kV = 0;
      wheelMotor_Slot0.kA = 0;
      wheelMotor_Slot0.kP = 0.3;
      wheelMotor_Slot0.kI = 0;
      wheelMotor_Slot0.kD = 0;

      //MotioinMagic Config
      turnMotionMagicConfigs.MotionMagicCruiseVelocity = 40;
      turnMotionMagicConfigs.MotionMagicAcceleration = 80;
      turnMotionMagicConfigs.MotionMagicJerk = 400;

      wheelMotionMagicConfig.MotionMagicAcceleration = 400;
      wheelMotionMagicConfig.MotionMagicJerk = 4000;
      // Absolute Encoder Configurations
      CANcoderConfig = new CANcoderConfiguration();
      CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      CANcoderConfig.MagnetSensor.MagnetOffset = WristConstants.encoderOffset;
      pivotCANcoder.getConfigurator().apply(CANcoderConfig);
  
      //Motor Configurations
      wheelMotor.getConfigurator().apply(wheelConfig);
      wheelMotor.getConfigurator().apply(wheelMotionMagicConfig);
      wheelMotor.getConfigurator().apply(wheelMotor_Slot0);
      pivotMotor.getConfigurator().apply(pivotConfig);
  
      // PID Controller and Feedforward
      armPID = new PIDController(WristConstants.Kp, WristConstants.Ki, WristConstants.Kd);
      armFeedforward = new ArmFeedforward(0, WristConstants.Kg1, 0);
      // Init debouncer
      m_Debouncer_first = new Debouncer(0.05, DebounceType.kRising);
      m_Debouncer_second = new Debouncer(0.05, DebounceType.kRising);
    }
  
    // ======== Arm ========
    // Intake coral
    public void intakeCoral_Arm() {
      pivotState = "ARM_INTAKECORAL";
      goalAngle = WristConstants.coralStationAngle;}
    public void Arm_IDLE() {
      pivotState = "ARM_IDLE";
      goalAngle = WristConstants.primitiveAngle;}
    public void Arm_shootCoral_L1() {
      pivotState = "ARM_SHOOTCORAL_L1"; // Update state for logging
      goalAngle = WristConstants.coralL1Angle;}
    public void Arm_shootCoral_L2() {
      pivotState = "ARM_SHOOTCORAL_L2"; // Update state for logging
      goalAngle = WristConstants.coralL2Angle;}
    public void Arm_shootCoral_L3() {
      pivotState = "ARM_SHOOTCORAL_L3"; // Update state for logging
      goalAngle = WristConstants.coralL3Angle;}
    public void Arm_shootCoral_L4() {
      pivotState = "ARM_SHOOTCORAL_L4"; // Update state for logging
      goalAngle = WristConstants.coralL4Angle;}
    public void coralL4Primitive_Arm(){
      pivotState = "ARM_CORAL_L4_PRIMITIVE"; // Update state for logging
      goalAngle = WristConstants.coralL4UpAngle;}

    public void Arm_NET_IDLE() {
      pivotState = "ARM_NET_IDLE"; // Update state for logging
      goalAngle = WristConstants.netUpAngle;}
    public void Arm_shootAlgae_NET() {
      pivotState = "ARM_SHOOTALGAE_NET"; // Update state for logging
      goalAngle = WristConstants.algaeNetAngle;}
    public void Arm_shootAlgae_Processor() {
      pivotState = "ARM_SHOOTALGAE_PROCESSOR"; // Update state for logging
      goalAngle = WristConstants.algaeProccesorAngle;}
    // Intake algae
    public void Arm_intakeAlgae_Low() {
      pivotState = "ARM_INTAKEALGAE_LOW"; // Update state for logging
      goalAngle = WristConstants.algaeLowInAngle;}
    public void Arm_intakeAlgae_High() {
      pivotState = "ARM_INTAKEALGAE_HIGH"; // Update state for logging
      goalAngle = WristConstants.algaeHighInAngle;}
    public void Arm_intakeAlgae_Floor() {
      pivotState = "ARM_INTAKEALGAE_FLOOR"; // Update state for logging
      goalAngle = WristConstants.algaeFloorAngle;}

    public void primitiveArm_HasCoral() {
      pivotState = "ARM_PRIMITIVE_HAS_CORAL"; // Update state for logging
      goalAngle = WristConstants.primitiveAngle_HasCoral;}
    public void Arm_RemoveAlgae() {
      pivotState = "ARM_REMOVEALGAE"; // Update state for logging
      goalAngle = WristConstants.algaeRemoveAngle;}

    // ======== Wheel ========
    // Inatake coral
    public void intakeCoral_Wheel() {
      wheelState = "WHEEL_INTAKECORAL";
      wheelMotor.setControl(requst_wheelSpeed.withVelocity(EndEffectorConstants.coralInSpeed_RotionPerSecond));}
    public void intakeCoralSlow_Wheel() {
      wheelState = "WHEEL_INTAKECORAL_SLOW"; 
      wheelMotor.setControl(requst_wheelSpeed.withVelocity(EndEffectorConstants.coralInSpeedSlow_RotationPerSecond));}
    // Coral
    public void turnMore_Coral() {
      wheelState = "WHEEL_TURN_MORE_CORAL";
      wheelMotor.setVoltage(EndEffectorConstants.coralTurnMore);}
    // Shoot coral
    public void Wheel_shootCoral_L1() {
      wheelState = "WHEEL_SHOOTCORAL_L1"; 
      wheelMotor.setVoltage(EndEffectorConstants.coralL1OutVol);}
    public void Wheel_shootCoral_L2() {
      wheelState = "WHEEL_SHOOTCORAL_L2"; 
      wheelMotor.setVoltage(EndEffectorConstants.coralL2OutVol);}
    public void Wheel_shootCoral_L3() {
      wheelState = "WHEEL_SHOOTCORAL_L3";
      wheelMotor.setVoltage(EndEffectorConstants.coralL3OutVol);}
    public void Wheel_shootCoral_L4() {
      wheelState = "WHEEL_SHOOTCORAL_L4";
      wheelMotor.setVoltage(EndEffectorConstants.coralL4OutVol);}
    // Intake Algae
    public void intakeAlgae_Low_Wheel() {
      wheelState = "WHEEL_INTAKEALGAE_LOW";
      wheelMotor.setVoltage(EndEffectorConstants.algaeLowInVol);}
    public void intakeAlgae_High_Wheel() {
      wheelState = "WHEEL_INTAKEALGAE_HIGH";
      wheelMotor.setVoltage(EndEffectorConstants.algaeHighInVol);}
    public void intakeAlgae_Floor_Wheel() {
      wheelState = "WHEEL_INTAKEALGAE_FLOOR";
      wheelMotor.setVoltage(EndEffectorConstants.algaeFloorInVol);}
    // Shoot Algae
    public void Wheel_shootAlgae_NET() {
      wheelState = "WHEEL_SHOOTALGAE_NET";
      wheelMotor.setVoltage(EndEffectorConstants.algaeShootNetVol);}
    public void Wheel_shootAlgae_Processor() {
      wheelState = "WHEEL_SHOOTALGAE_PROCESSOR";
      wheelMotor.setVoltage(EndEffectorConstants.algaeShootProcessorVol);}
    // Wheel control
    public void outAlgae() {
      wheelState = "WHEEL_OUTALGAE";
      wheelMotor.setVoltage(EndEffectorConstants.algaeOutVol);}
    public void holdAlgae() {
      wheelState = "WHEEL_HOLDALGAE";
      wheelMotor.setVoltage(EndEffectorConstants.algaeHoldVol);}
    public void RemoveAlgae() {
      wheelState = "WHEEL_REMOVEALGAE";
      wheelMotor.setVoltage(EndEffectorConstants.algaeRemoveVol);}
    public void stopWheel() {
      wheelState = "WHEEL_STOP";
      wheelMotor.setControl(requst_wheelSpeed.withVelocity(0));}

    
    public double getPosition() {
      return pivotMotor.getPosition().getValueAsDouble();
    }
  
    public double getAbsolutePosition() {
      return pivotCANcoder.getAbsolutePosition().getValueAsDouble();
    }
  
    public double getAngle() {
      return pivotCANcoder.getAbsolutePosition().getValueAsDouble()*360;
    }
  
    public double getRadians() {
      return Units.degreesToRadians(getAngle());
    }
  
    public double getVelocity() {
      return Units.rotationsPerMinuteToRadiansPerSecond(pivotCANcoder.getVelocity().getValueAsDouble()*60);
    }
    
    // IR sensor
    public boolean getFirstIR() {
      return m_Debouncer_first.calculate(irSensor_CoralFirst.get());
    }
    public boolean getSecondIR() {
      return m_Debouncer_second.calculate(irSensor_CoralSecond.get());
    }

    public boolean getAlgaeIR() {
      return irSensor_Algae.get();
    }

    public boolean shouldCoralSlow() {
      return !getFirstIR() && !getSecondIR();
    }

    public boolean canMoveUp() {
      return getFirstIR();
    }

    public boolean hasCoral() {
      return (getFirstIR()) && (!getSecondIR());
    }

    public boolean hasAlgae() {
      return !irSensor_Algae.get();
    }

    public boolean wheelOverCurrent(){
      return Math.abs(wheelMotor.getStatorCurrent().getValueAsDouble()) > 40;
    }
  
    public boolean arrivedSetpoint() {
      return (Math.abs(armPID.getError()) <= 2);
    }
  
  
    @Override
    public void periodic() {
      // Arm feedforward
      if(90 >= getAngle() && getAngle() > 80 || 75 >= getAngle() && getAngle() > 70) {
        armFeedforward = new ArmFeedforward(0, WristConstants.Kg1, 0);
      }else if(80 >= getAngle() && getAngle() > 75) {
        armFeedforward = new ArmFeedforward(0, WristConstants.Kg4, 0);
      }else if(70 >= getAngle() && getAngle() > 61.6){
        armFeedforward = new ArmFeedforward(0, WristConstants.Kg2, 0);
      }else {
        armFeedforward = new ArmFeedforward(0, WristConstants.Kg3, 0);
      }
      
      // PID and Feedforward
      pidOutput = armPID.calculate(getAngle(), goalAngle);
      feedforwardOutput = armFeedforward.calculate(getRadians(), getVelocity())/12;
      pidOutput = Constants.setMaxOutput(pidOutput, WristConstants.PIDMaxOutput);
      // Implement
      output = pidOutput + feedforwardOutput;
      pivotMotor.set(output);

      //Log
      // Wrist
      SmartDashboard.putNumber("Wrist/PidOutput", pidOutput);
      SmartDashboard.putNumber("Wrist/FeedforwardOutput", feedforwardOutput);
      SmartDashboard.putNumber("Wrist/Output", output);
      SmartDashboard.putBoolean("Wrist/arrivedSetpoint", arrivedSetpoint());
      SmartDashboard.putNumber("Wrist/PivotAngle", getAngle());
      SmartDashboard.putNumber("Wrist/PivotSetpoint", goalAngle);
      SmartDashboard.putString("Wrist/State", pivotState);
      // Wheel
      SmartDashboard.putString("EndEffector/State", wheelState);
      SmartDashboard.putNumber("EndEffector/WheelCurrent", wheelMotor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putBoolean("EndEffector/FirstIR", getFirstIR());
      SmartDashboard.putBoolean("EndEffector/SecondIR", getSecondIR());
      SmartDashboard.putBoolean("EndEffector/AlgaeIR", getAlgaeIR());
      SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
      SmartDashboard.putBoolean("EndEffector/hasAlgae", hasAlgae());
      
  }
}
