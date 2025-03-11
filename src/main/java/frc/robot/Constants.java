// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
//import swervelib.math.Matter;

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
 // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class FalconMotorConstants {
    public static final double kFreeSpeedRpm = 6380, 
    stallCurrent = 257 /* Amp */,
    stallTorque = 4.69 /* Newton-m */;
  }
 
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kMinSpeedMultiplier = 0.5;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5); //values have been adjusted for current bot
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);  //values have been adjusted for current bot
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    //Falcon CAN IDs
    //All CAN IDs set for current bot
    public static final int kFrontLeftDrivingCanId = 2; 
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 6;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = FalconMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = ((45.0 * 22) / (kDrivingMotorPinionTeeth * 15))/ kWheelCircumferenceMeters;
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps )
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMps;
    public static final double kDrivingP = 0.9 * kDrivingFF;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

  
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
 public static class OperatorConstants
  {
    public static final int kCtrl1 = 0;
    public static final int kCtrl2 = 1;
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double kDriveDeadband = 0.05;

    // wrist to slide interlock:
    public static final double safeToSlide = .5;//TODO
  }

  //Put new Constant classes here.
  public static class ElevatorConstants {
    public static final int LMotorID = 9;
    public static final int RMotorID = 10;
    public static final double Level1 = 3.390; //Inches.
    public static final double Level2 = 4.226;
    public static final double Level3 = 8.224;
    public static final double Level4 = 16.021;
    public static final double LevelMAX = 19.875;
	  public static final double kP = .1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double gearing = 20/1,
      inchPerRotation = 25/4;
    public static final double maxVelocity = 0, // in/sec
      maxAcceleration = 0, // in/sec^2
      gravityCompensator = 0; // Volts
	
  }

  public static class AlgaeConstants {
    public static final int leftMotorID = 13;
    public static final int rightMotorID = 14;
    public static final double motorVelocity = /* 2000 */12 /*V */;
    public static final double kP = .0001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double gearRatio = 25/1;
    public static final double highCurrent = 5 /*A */;
    public static final double highCurrentTime = .25;/*sec*/
  }

  public static class CoralConstants {
    public static final int coralMotorID = 15;
    public static final double motorVelocity = /* 2000 */12 /*V */;
    public static final double kP = .0001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double gearRatio = 25/1;
    public static final double highCurrent = 3 /*A */;
    public static final double highCurrentTime = .25;/*sec*/  
  }

  public static class SlideConstants {

    public static final int motorID = 12;
    public static final double kP = .4;
    public static final double gearRatio = 25/1;
    public static final double screwRatio = 1/1; // in / rotation
	  public static final double centerPos = 0;
    public static final double width = 13.75;
    public static final double leftPos = -width/2; //inches
    public static final double rightPos = width/2;
  
  }

  public static class WristConstants {
    public static final int wristMotorID = 11;
    public static final int absEncoderID = 13;
    public static final double motorVelocity = 2000;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double gravityCompensation = 0; //This will need to be changed based on how testing goes. 
    public static final double gearRatio = 25/1; //This needs to be updated, this isn't correct for the motor that we have on the bot. 
    //All of these positions are temporarry, they were measured on the absolute encoder but will need tuning later. 
    //All measurements were taken in volts and multiplied by 360 to get the angle in degrees.
    public static final double coralCollectionPosition = 0;  
    public static final double bargePosition = 0.165 * 360; //Turns volts to angles
    public static final double L4Position = 0.5 * 360; //Turns volts to angles
    public static final double L23Position = 0.396 * 360; //Turns volts to angles
    public static final double algaeIntake = 0.34 * 360;  //Turns volts to angles
  }

 
  public static enum Positions {
    ALFA_BRAVO,
    CHARLIE_DELTA,
    ECHO_FOXTROT,
    GOLF_HOTEL,
    INDIA_JULIET,
    KILO_LIMA,
    BARGE,
    LEFT_STATION,
    RIGHT_STATION
  }

}
 
