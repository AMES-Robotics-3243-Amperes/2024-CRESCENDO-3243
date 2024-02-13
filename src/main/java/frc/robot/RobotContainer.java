// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoyUtilConstants;
import frc.robot.commands.automatics.CommandPickupFieldNote;
import frc.robot.commands.automatics.CommandScoreInAmp;
import frc.robot.commands.automatics.CommandScoreInSpeaker;
import frc.robot.commands.climber.CommandClimberTeleop;
import frc.robot.commands.drivetrain.CommandSwerveTeleopDrive;
import frc.robot.commands.intake.CommandIntakeTeleop;
import frc.robot.commands.plate.CommandPlateTeleop;
import frc.robot.commands.shooter.CommandShooterStop;
import frc.robot.commands.shooter.CommandShooterTeleopAmp;
import frc.robot.commands.shooter.CommandShooterTeleopSpeaker;
import frc.robot.subsystems.SubsystemPhotonvision;
import frc.robot.subsystems.SubsystemPlate;
import frc.robot.subsystems.SubsystemShooter;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemClimber;
import java.io.IOException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final SubsystemShooter m_SubsystemShooter = new SubsystemShooter();
  private final SubsystemClimber m_SubsystemClimber = new SubsystemClimber();
  //private final SubsystemPlate m_subsystemPlate = new SubsystemPlate();

  //
  // :3 COMMANDS
  //


  private final CommandSwerveTeleopDrive m_CommandSwerveTeleopDrive = new CommandSwerveTeleopDrive(m_SubsystemSwerveDrivetrain, primaryController);
  private final CommandIntakeTeleop m_teleopCommandIntake = new CommandIntakeTeleop(m_subsystemIntake, secondaryController);
  private final CommandShooterTeleopAmp m_CommandShooterTeleopAmp = new CommandShooterTeleopAmp(m_SubsystemShooter);
  private final CommandShooterTeleopSpeaker m_CommandShooterTeleopSpeaker = new CommandShooterTeleopSpeaker(m_SubsystemShooter);
  //private final CommandShooterStop m_CommandShooterStop = new CommandShooterStop(m_SubsystemShooter);

  private final CommandClimberTeleop m_CommandClimberTeleop = new CommandClimberTeleop(m_SubsystemClimber, primaryController);
  //private final CommandPlateTeleop m_commandPlateTeleop = new CommandPlateTeleop(m_subsystemPlate, secondaryController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SubsystemSwerveDrivetrain.setDefaultCommand(m_CommandSwerveTeleopDrive);
    m_subsystemIntake.setDefaultCommand(m_teleopCommandIntake);
    m_SubsystemClimber.setDefaultCommand(m_CommandClimberTeleop);
    //m_subsystemPlate.setDefaultCommand(m_commandPlateTeleop);
    /* 
    try {
      new SubsystemPhotonvision();
    } catch (IOException e) {
      System.out.println("ruh roh, camera moment");
    }
    */
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
  private void configureBindings() {

    // && Toggle amp shooting
    secondaryController.x().toggleOnTrue(m_CommandShooterTeleopAmp);
    // && toggle speaker shooting
    secondaryController.y().toggleOnTrue(m_CommandShooterTeleopSpeaker);

    primaryController.a().whileTrue(new CommandScoreInAmp(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter));
    primaryController.b().whileTrue(new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new CommandPickupFieldNote(m_SubsystemSwerveDrivetrain, m_subsystemIntake, 0),
      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter),
      new CommandPickupFieldNote(m_SubsystemSwerveDrivetrain, m_subsystemIntake, 1),
      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter)
    );
  }
}
