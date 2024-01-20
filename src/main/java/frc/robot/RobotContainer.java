// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoyUtilConstants;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandTeleopIntake;
import frc.robot.subsystems.SubsystemPhotonvision;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemClimber;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandClimber;
import frc.robot.test.ExampleTestGroup;

import java.io.IOException;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //
  // :3 Controller
  //
  private final JoyUtil primaryController = new JoyUtil(JoyUtilConstants.primaryControllerID);
  private final JoyUtil secondaryController = new JoyUtil(JoyUtilConstants.secondaryControllerID);
  
  //
  // :3 SUBSYSTEMS
  //
  private final SubsystemSwerveDrivetrain m_SubsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  private final SubsystemIntake m_subsystemIntake = new SubsystemIntake();
  
  private final SubsystemClimber m_SubsystemClimber = new SubsystemClimber();
  //private final ExampleTestGroup m_ExampleTestGroup = new ExampleTestGroup();

  //
  // :3 COMMANDS
  //
  private final CommandSwerveTeleopDrive m_CommandSwerveTeleopDrive = new CommandSwerveTeleopDrive(m_SubsystemSwerveDrivetrain, primaryController);
  public final CommandTeleopIntake m_teleopCommandIntake = new CommandTeleopIntake(m_subsystemIntake, secondaryController);
  private final CommandClimber m_CommandClimber = new CommandClimber(m_SubsystemClimber, primaryController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SubsystemSwerveDrivetrain.setDefaultCommand(m_CommandSwerveTeleopDrive);
    m_subsystemIntake.setDefaultCommand(m_teleopCommandIntake);
    try {
      new SubsystemPhotonvision();
    } catch (IOException e) {
      System.out.println("ruh roh, camera moment");
    }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
