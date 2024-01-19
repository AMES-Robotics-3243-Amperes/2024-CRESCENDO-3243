// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class SubsystemIntake extends SubsystemBase {

private final CANSparkMax intakeMotor1;

protected final RelativeEncoder IntakeEncoder;

protected final SparkPIDController IntakePID;

  /** Creates a new IntakeSubsystem. */
  public SubsystemIntake() {
    intakeMotor1 = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    IntakeEncoder = intakeMotor1.getEncoder();
    IntakePID = intakeMotor1.getPIDController();
    IntakePID.setFeedbackDevice(IntakeEncoder);

    IntakePID.setP(kP);
    IntakePID.setI(kI);
    IntakePID.setD(kD);
    IntakePID.setFF(kFF);

    IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void TurnOnIntake() {
    IntakePID.setReference(kV, CANSparkMax.ControlType.kVelocity);
  }

  public void TurnOffIntake() {
    IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
