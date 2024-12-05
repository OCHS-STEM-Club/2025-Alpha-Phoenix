// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.05; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(1, 0, 0.01);
  }


  public static class DriveConstants
  {
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
    public static final double MAX_ANGULAR_RATE = 1.5 * Math.PI;

    
    
    // Joystick Deadband
    public static final double TRANSLATION_DEADBAND  = 0.1;
    public static final double ROTATION_DEADBAND = 0.1;
    
    public static final double TRANSLATION_CONSTANT = 1;
    public static final double ROTATION_CONSTANT = 1;
    

    public static final int kOperatorControllerPort = 2;
    public static final int kDriverControllerPort = 0;

    public static final boolean isButtonBoxBeingUsed = true;
  }

 
  
}