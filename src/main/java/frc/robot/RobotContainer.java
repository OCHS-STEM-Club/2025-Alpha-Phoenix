// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  
  private SendableChooser<Double> speedChooser = new SendableChooser<>();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort); 

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  // Slew rate limiters
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.TRANSLATION_DEADBAND)
      .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                                                               
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

  public Double lastSpeed = 0.0;

  private void configureBindings() {
    
    newSpeed();

    Trigger speedPick = new Trigger(() -> lastSpeed != speedChooser.getSelected());
    speedPick.onTrue(Commands.runOnce(() -> newSpeed()));
    
    m_driverController.leftBumper().onFalse(Commands.runOnce(() -> DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected()));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(yLimiter.calculate(m_driverController.getLeftY() * DriveConstants.MAX_SPEED))
            .withVelocityY(xLimiter.calculate(m_driverController.getLeftX() * DriveConstants.MAX_SPEED)) 
            .withRotationalRate(rotLimiter.calculate(-m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE)) 
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
    // newSpeed();
    // Adjustable speed Chooser
    speedChooser.addOption("100%", 1.0);
    speedChooser.addOption("95%", 0.95);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("85%", 0.85);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("75%", 0.75);
    speedChooser.addOption("70%", 0.7);
    speedChooser.addOption("65%", 0.65);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("55%", 0.55);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("45%", 0.45);
    speedChooser.addOption("40%", 0.4);
    speedChooser.addOption("35%", 0.35);
    speedChooser.addOption("30%", 0.3);
    speedChooser.addOption("25%", 0.25);
    speedChooser.setDefaultOption("0%", 0.0);
    // Put chooser on dashboard
    SmartDashboard.putData("Speed Limit", speedChooser);

    configureBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void newSpeed() {
    lastSpeed = speedChooser.getSelected();
    DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
