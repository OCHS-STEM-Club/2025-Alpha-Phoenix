// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new coralIntakeSubsystem. */
  private CANSparkMax coralIntake;
  public CoralIntakeSubsystem() {
    coralIntake = new CANSparkMax(18, MotorType.kBrushless);
    coralIntake.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Intake Motor", coralIntake.get());

  }
  
  public void intakeIn() {
    coralIntake.set(0.2);
  }
  public void intakeOff() {
    coralIntake.set(0);
  }
  public void intakeOut() {
    coralIntake.set(-0.2);
  }
}
