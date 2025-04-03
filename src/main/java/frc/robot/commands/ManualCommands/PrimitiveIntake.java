// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class PrimitiveIntake extends Command {
  private final ElevatorSubsystem m_Elevator;
  private final EndEffectorSubsystem m_EndEffector;

  public PrimitiveIntake(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    this.m_Elevator = elevatorSubsystem;
    this.m_EndEffector = endEffectorSubsystem;
    addRequirements(m_Elevator, m_EndEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Has Algae
    if(m_EndEffector.hasAlgae()) {
      // Reset the end effector
      m_EndEffector.Arm_IDLE();
      m_EndEffector.holdAlgae();
      // Reset the elevator
      if(m_EndEffector.arrivedSetpoint()) m_Elevator.toPrimitive();
      // Reset the LED
      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.hasGamePiece = true;
      LEDConstants.LEDFlag = true;
    }else if(m_EndEffector.hasCoral() && Math.abs(m_Elevator.getPositionRot() - ElevatorConstants.coralL4Position) <= 2) {
      // Move the end effector
      m_EndEffector.primitiveArm_HasCoral();
      m_EndEffector.stopWheel();
      // Move the elevator
      if(m_EndEffector.arrivedSetpoint()) {
        m_Elevator.toPrimitive();
        // Reset the end effector
        if(m_Elevator.arriveSetPoint()) {
          m_EndEffector.Arm_IDLE();
          
          LEDConstants.intakeArriving = false;
          LEDConstants.arrivePosition_Intake = false;
          LEDConstants.arrivePosition_Base = false;
          LEDConstants.tracking = false;
          LEDConstants.LEDFlag = true;
        }
      }
    }else {
      // Reset end effector and elevator
      m_EndEffector.Arm_IDLE();
      m_EndEffector.stopWheel();
      if(m_EndEffector.arrivedSetpoint()) m_Elevator.toPrimitive();
    
      LEDConstants.intakeArriving = false;
      LEDConstants.arrivePosition_Intake = false;
      LEDConstants.arrivePosition_Base = false;
      LEDConstants.tracking = false;
      LEDConstants.hasGamePiece = false;
      LEDConstants.LEDFlag = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    LEDConstants.hasAlgae = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
