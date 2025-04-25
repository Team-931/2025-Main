// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    //LimelightHelpers.setStreamMode_PiPSecondary("limelight-right");
  }
  Binocular binoc = new Binocular();
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("left targets", LimelightHelpers.getTargetCount("limelight-left"));
    SmartDashboard.putNumber("left id", LimelightHelpers.getFiducialID("limelight-left"));
    SmartDashboard.putNumber("left x", LimelightHelpers.getTX("limelight-left"));
    
    SmartDashboard.putNumber("right targets", LimelightHelpers.getTargetCount("limelight-right"));
    SmartDashboard.putNumber("right id", LimelightHelpers.getFiducialID("limelight-right"));
    SmartDashboard.putNumber("right x", LimelightHelpers.getTX("limelight-right"));
    binoc.readTargets();
    SmartDashboard.putNumber("distance", binoc.forwardDist(1));
    SmartDashboard.putNumber("right dist", binoc.getRightDist(1));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // CameraServer.startAutomaticCapture();
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
