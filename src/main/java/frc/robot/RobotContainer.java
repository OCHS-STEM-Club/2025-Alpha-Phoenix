// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort); 

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.TRANSLATION_DEADBAND)
      .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
                                                               
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

  private void configureBindings() {
    
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(m_driverController.getLeftY() * DriveConstants.MAX_SPEED)
            .withVelocityY(m_driverController.getLeftX() * DriveConstants.MAX_SPEED) 
            .withRotationalRate(-m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE) 
            ));

      m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    
      m_driverController.b().whileTrue(drivetrain
          .applyRequest(() -> point.withModuleDirection(
          new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
      m_driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

      if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      }
      drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
