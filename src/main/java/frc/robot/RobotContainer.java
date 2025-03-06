// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.PoseEstimator;

public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final CommandXboxController driver, operator;
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final Drivetrain drivetrain;
  private final PoseEstimator poseEstimator;

  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(0);
    drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),"swerve"));
    poseEstimator = new PoseEstimator(drivetrain);
     
    // TODO: add operator subsystems.
    configureBindings();

    drivetrain.setDefaultCommand(drivetrain.driveCommand(()-> driver.getLeftX(), ()-> driver.getLeftY(), ()-> driver.getRightX()));
  }

  private void configureBindings() {
    //operator.button(8).onTrue(algaeIntake.in());
    //operator.axisLessThan(2, -.5).onTrue(algae center);
    operator.button(11).onTrue(elevator.runOnce(()->elevator.SetLevel(2)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
