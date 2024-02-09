// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemShuffleboard extends SubsystemBase {
  /** Creates a new SubsystemShuffleboard. */
  public SubsystemShuffleboard() {

    ShuffleboardTab tabMain = Shuffleboard.getTab("Spicy");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
