// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ManualDrive_Kraken;
import frc.robot.commands.ManualDrive_RotationSpeedUp;
import frc.robot.commands.AutoCommand.Coral_L1_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L2_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L3_Elevator_Auto;
import frc.robot.commands.AutoCommand.Coral_L4_Elevator_Auto;
import frc.robot.commands.AutoCommand.IntakeAlgae_High_Auto;
import frc.robot.commands.AutoCommand.IntakeAlgae_Low_Auto;
import frc.robot.commands.AutoCommand.IntakeCoral_Fast;
import frc.robot.commands.AutoCommand.IntakeCoral_IDLE;
import frc.robot.commands.AutoCommand.IntakeCoral_Slow;
import frc.robot.commands.AutoCommand.NET_Elevator;
import frc.robot.commands.AutoCommand.PrepareForScore_Algae_Auto;
import frc.robot.commands.AutoCommand.PrepareForScore_Coral_Auto;
import frc.robot.commands.AutoCommand.PrimitiveIntake_Auto;
import frc.robot.commands.AutoCommand.ShootCoral_Auto;
import frc.robot.commands.AutoCommand.TrackLeftReef_Auto;
import frc.robot.commands.AutoCommand.TrackMiddleReef_Auto;
import frc.robot.commands.AutoCommand.TrackRightReef_Auto;
import frc.robot.commands.ClimbCommand.Climb;
import frc.robot.commands.ClimbCommand.PrepClimb;
import frc.robot.commands.ClimbCommand.ResetClimber;
import frc.robot.commands.ManualCommands.Coral_L1;
import frc.robot.commands.ManualCommands.Coral_L2;
import frc.robot.commands.ManualCommands.Coral_L3;
import frc.robot.commands.ManualCommands.IntakeAlgae_Floor;
import frc.robot.commands.ManualCommands.IntakeAlgae_High;
import frc.robot.commands.ManualCommands.IntakeAlgae_Low;
import frc.robot.commands.ManualCommands.IntakeCoral;
import frc.robot.commands.ManualCommands.OutAlgae;
import frc.robot.commands.ManualCommands.PrimitiveIntake;
import frc.robot.commands.ManualCommands.ShootNet;
import frc.robot.commands.ManualCommands.ShootProcessor;
import frc.robot.commands.ManualCommands.TurnMore;
import frc.robot.commands.TestCommand.Coral_L4Test;
import frc.robot.commands.TrackCommands.TrackLeftReef;
import frc.robot.commands.TrackCommands.TrackMiddleReef_Left;
import frc.robot.commands.TrackCommands.TrackMiddleReef_Right;
import frc.robot.commands.TrackCommands.TrackRightReef;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem_Kraken;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem = new PhotonVisionSubsystem();
  private final SwerveSubsystem_Kraken m_SwerveSubsystem = new SwerveSubsystem_Kraken();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final EndEffectorSubsystem m_EndEffectorSubsystem = new EndEffectorSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); 

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("stopMotor", Commands.runOnce(() -> m_SwerveSubsystem.stopMotor(), m_SwerveSubsystem));
    NamedCommands.registerCommand("TrackLeftReef_Auto", new TrackLeftReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(2.5));
    NamedCommands.registerCommand("TrackRightReef_Auto", new TrackRightReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(2.5));//1
    NamedCommands.registerCommand("TrackLeftReef_Auto_OneCoral", new TrackLeftReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(3));
    NamedCommands.registerCommand("TrackRightReef_Auto_OneCoral", new TrackRightReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(3));
    NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(5));
    NamedCommands.registerCommand("IntakeCoral_fast", new IntakeCoral_Fast(m_EndEffectorSubsystem));
    NamedCommands.registerCommand("IntakeCoral_Slow", new IntakeCoral_Slow(m_EndEffectorSubsystem));
    NamedCommands.registerCommand("PrimitiveIntake", new PrimitiveIntake_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.4));
    NamedCommands.registerCommand("Coral_L4_Intake_WithTrack", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(1));
    NamedCommands.registerCommand("Coral_L4_Intake", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.8));
    NamedCommands.registerCommand("Coral_L4_Intake_WithTrack_OneCoral", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(2));
    NamedCommands.registerCommand("Coral_L4_Intake_OneCoral", new Coral_L4_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(2));
    NamedCommands.registerCommand("Coral_L2_Intake", new Coral_L2_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.2));
    NamedCommands.registerCommand("Coral_L3_Intake", new Coral_L3_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(1.5));
    NamedCommands.registerCommand("Coral_L1_Intake", new Coral_L1_Elevator_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.1));
    NamedCommands.registerCommand("IntakeCoral_IDLE", new IntakeCoral_IDLE(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(0.8));
    NamedCommands.registerCommand("TrackMiddleReef_Auto", new TrackMiddleReef_Auto(m_PhotonVisionSubsystem, m_SwerveSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("IntakeHighAlgae_Auto", new IntakeAlgae_High_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(1));
    NamedCommands.registerCommand("IntakeLowAlgae_Auto", new IntakeAlgae_Low_Auto(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(1));
    NamedCommands.registerCommand("PrepareForScore_Net", new PrepareForScore_Algae_Auto(m_EndEffectorSubsystem, m_ElevatorSubsystem));
    NamedCommands.registerCommand("PrepareForScore_Coral", new PrepareForScore_Coral_Auto(m_EndEffectorSubsystem, m_ElevatorSubsystem));
    NamedCommands.registerCommand("NET_Intake_Auto", new NET_Elevator(m_ElevatorSubsystem, m_EndEffectorSubsystem).withTimeout(1));
    NamedCommands.registerCommand("OutAlgae_Auto", new OutAlgae(m_EndEffectorSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("TrackMiddle_Right_Auto", new TrackMiddleReef_Right(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    NamedCommands.registerCommand("TrackMiddle_Left_Auto", new TrackMiddleReef_Left(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    NamedCommands.registerCommand("ShootCoral_Auto", new ShootCoral_Auto(m_EndEffectorSubsystem).withTimeout(0.4));
    NamedCommands.registerCommand("ResetGyro", Commands.runOnce(()->{m_SwerveSubsystem.resetGyro();}));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }


  private void configureBindings() {
    // Driver Controller
    DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
    DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
    DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);

    BooleanSupplier isSlowFunc = ()-> driverController.getHID().getRightTriggerAxis() > 0.2;
    BooleanSupplier ifFeed = ()-> driverController.getHID().getLeftTriggerAxis() > 0.2;
    BooleanSupplier ifClimb = ()-> driverController.getHID().getPOV() == 0; // Use the D-Pad to indicate if climbing, this can be changed to a button if needed

    driverController.leftBumper().whileTrue(new TrackLeftReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    // driverController.leftTrigger(0.4).toggleOnTrue(new TrackMiddleReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.rightBumper().whileTrue(new TrackRightReef(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    // driverController.a().toggleOnTrue(new TrackCage(m_SwerveSubsystem, m_PhotonVisionSubsystem));
    // driverController.pov(270).whileTrue(new TrackMiddleReef_Left(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    // driverController.pov(180).whileTrue(new TrackMiddleReef_Right(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.pov(270).whileTrue(new TrackMiddleReef_Left(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.pov(90).whileTrue(new  ManualDrive_RotationSpeedUp(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
    driverController.a().whileTrue(new ResetClimber(m_ClimberSubsystem, ifClimb));
    driverController.b().toggleOnTrue(new PrepClimb(m_ClimberSubsystem, ifClimb));
    driverController.x().whileTrue(new Climb(m_ClimberSubsystem, ifClimb));

    driverController.y().whileTrue(
      Commands.runOnce(()->{
        m_SwerveSubsystem.resetGyro();
      })
    );


    // Operator Controller
    operatorController.pov(180).toggleOnTrue(new Coral_L1(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(0).toggleOnTrue(new Coral_L2(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.leftTrigger().toggleOnTrue(new Coral_L3(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.leftBumper().toggleOnTrue(new Coral_L4Test(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(270).toggleOnTrue(new ShootProcessor(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.pov(90).toggleOnTrue(new IntakeAlgae_Floor(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.rightTrigger().toggleOnTrue(new IntakeAlgae_Low(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.rightBumper().toggleOnTrue(new IntakeAlgae_High(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    
    // operatorController.x().toggleOnTrue(new RemoveAlgae(m_EndEffectorSubsystem, m_ElevatorSubsystem));
    // operatorController.x().toggleOnTrue(new PrepareForScore_Coral_Auto(m_EndEffectorSubsystem, m_ElevatorSubsystem));
    operatorController.a().toggleOnTrue(new PrimitiveIntake(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.b().toggleOnTrue(new IntakeCoral(m_ElevatorSubsystem, m_EndEffectorSubsystem));
    operatorController.y().toggleOnTrue(new ShootNet(m_ElevatorSubsystem, m_EndEffectorSubsystem, ifFeed));
    operatorController.axisGreaterThan(5, 0.6).whileTrue(new OutAlgae(m_EndEffectorSubsystem));
    operatorController.axisGreaterThan(1, 0.6).whileTrue(new TurnMore(m_EndEffectorSubsystem));

    // Set the default drive command 
    m_SwerveSubsystem.setDefaultCommand(new ManualDrive_Kraken(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
} 
