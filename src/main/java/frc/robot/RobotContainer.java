// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
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
import frc.robot.commands.Coral_Intake.Coral_Intake_Cmd;
import frc.robot.commands.Coral_Intake.Coral_Outake_Cmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class RobotContainer {

  // Orchestra Definition
  Orchestra m_orchestra = new Orchestra();
  AudioConfigs m_audioConfigs = new AudioConfigs();

  // Chooser definitions
  private final SendableChooser<Command> autoChooser;
  private SendableChooser<Double> speedChooser = new SendableChooser<>();

  // Controller definitions
  private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort); 

  // Subsystem definitions
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();

  // Commands definitions
  Coral_Intake_Cmd m_coralIntakeCmd = new Coral_Intake_Cmd(m_coralIntakeSubsystem);
  Coral_Outake_Cmd m_coralOutakeCmd = new Coral_Outake_Cmd(m_coralIntakeSubsystem);

  //  // Slew rate limiters
  // private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(DriveConstants.X_SLEW_RATE_LIMITER);
  // private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(DriveConstants.Y_SLEW_RATE_LIMITER);
  // private final SlewRateLimiter rotSlewLimiter = new SlewRateLimiter(DriveConstants.ROT_SLEW_RATE_LIMITER);

  // Drive requests
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.TRANSLATION_DEADBAND)
      .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Telemetry
  private final Telemetry logger = new Telemetry(DriveConstants.MAX_SPEED);

  

  private void configureBindings() {
    
    // New speed method called for adjustable speed chooser
    newSpeed();

    // Creep button
    m_driverController.leftBumper().onTrue(Commands.runOnce(() ->
     DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps * DriveConstants.CREEP_SPEED));
    
    //  Refresh button for speed chooser
    m_driverController.leftBumper().onFalse(Commands.runOnce(() ->
     DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps * speedChooser.getSelected()));


    // Drive binding with slew rate limiters
    // drivetrain.setDefaultCommand( 
    //     drivetrain.applyRequest(() -> 
    //         drive.withVelocityX(xSlewLimiter.calculate(m_driverController.getLeftY() * DriveConstants.MAX_SPEED))
    //          .withVelocityY(ySlewLimiter.calculate(m_driverController.getLeftX() * DriveConstants.MAX_SPEED)) 
    //          .withRotationalRate(rotSlewLimiter.calculate(-m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE)) 
    //         )); 

      // Binding drive to controls
      drivetrain.setDefaultCommand( 
            drivetrain.applyRequest(() -> 
                drive.withVelocityX((m_driverController.getLeftY() * DriveConstants.MAX_SPEED))
                .withVelocityY((m_driverController.getLeftX() * DriveConstants.MAX_SPEED)) 
                .withRotationalRate((-m_driverController.getRightX() * DriveConstants.MAX_ANGULAR_RATE)) 
                ));

      // Brake more or X wheels
      m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
  
      // Points wheel in direction of joystick
      m_driverController.b().whileTrue(drivetrain
          .applyRequest(() -> point.withModuleDirection(
          new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

      // Gyro reset
      m_driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

      // Simulation things
      if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      }
      drivetrain.registerTelemetry(logger::telemeterize);

      // Intake CMD
      m_driverController.leftTrigger().whileTrue(
        m_coralIntakeCmd
        );

      // Outake CMD
      m_driverController.rightTrigger().whileTrue(
        m_coralOutakeCmd
        );

      // Orchestra play bind
      m_driverController.rightBumper().onTrue(
        Commands.runOnce(this :: playMusic)
      );
  }

  public RobotContainer() {
    // Orchestra configs
    m_orchestra.addInstrument(drivetrain.getModule(1).getDriveMotor());
    m_orchestra.loadMusic("output.chrp");
    m_audioConfigs.AllowMusicDurDisable = true;
    

    // Build Auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();

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
    speedChooser.setDefaultOption("50%", 0.5);
    speedChooser.addOption("45%", 0.45);
    speedChooser.addOption("40%", 0.4);
    speedChooser.addOption("35%", 0.35);
    speedChooser.addOption("30%", 0.3);
    speedChooser.addOption("25%", 0.25);
    speedChooser.addOption("0%", 0.0);

    // Put chooser on dashboard
    SmartDashboard.putData("Speed Limit", speedChooser);

    // Configures all bindings
    configureBindings();

    // Puts auto chooser on dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // Method to refresh speed chooser
  private void newSpeed() {
    DriveConstants.LAST_SPEED = speedChooser.getSelected();
    DriveConstants.MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps * DriveConstants.LAST_SPEED;
  }

  // Orchestra play method
  public void playMusic(){
    m_orchestra.play();
  }

  // Method to return input from auto chooser
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


}
