// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.SwerveSubsystem_Kraken;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // private final SwerveSubsystem_Kraken m_SwerveSubsystem = new SwerveSubsystem_Kraken();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
    LEDConstants.fireAnimation = false;
    LEDConstants.normal = true;
    LEDConstants.LEDFlag = true;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
  }

  // // PathPlanner recommendations
  // @Override
  // public void robotInit() {
  //   FollowPathCommand.warmupCommand().schedule();
  // }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LEDConstants.fireAnimation = true;
    LEDConstants.LEDFlag = true;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    LEDConstants.fireAnimation = false;
    LEDConstants.LEDFlag = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Optional<Alliance> ally = DriverStation.getAlliance();
    // // If in red alliance, reset the gyro.
    // if (ally.isPresent()) {
    //   if (ally.get() == Alliance.Red) {
    //     double gyroAngle = m_SwerveSubsystem.getRotation().getDegrees();
    //     // if gryoAngle < 0, add 180. if gyroAngle > 0, subtract 180
    //     if (gyroAngle < 0) gyroAngle += 180;
    //     else gyroAngle -= 180;
    //     m_SwerveSubsystem.setGyroAngle(gyroAngle);
    //   }
    // }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LEDConstants.fireAnimation = false;
    LEDConstants.LEDFlag = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
