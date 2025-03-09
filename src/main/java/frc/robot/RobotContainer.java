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
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.CoralSlide;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.PoseEstimator;
import frc.robot.Subsystems.Wrist;

public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final Wrist wrist = new Wrist();
  private final CommandXboxController driver, operator;
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final CoralIntake coralIntake = new CoralIntake();
  private final CoralSlide coralSlide = new CoralSlide();
  private final Drivetrain drivetrain;
  private final PoseEstimator poseEstimator;

  public RobotContainer() {
    driver = new CommandXboxController(1);
    operator = new CommandXboxController(0);
    drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),"swerve"), driver);
    poseEstimator = new PoseEstimator(drivetrain);
     
    // TODO: add operator subsystems.
    configureBindings();

    drivetrain.setDefaultCommand(drivetrain.driveCommand(()-> driver.getLeftX(), ()-> driver.getLeftY(), ()-> driver.getRightX()));
  }

  private void configureBindings() {

    operator.button(3).onTrue(wrist.runOnce(() -> wrist.setWristPosition("collectCoral")));
    operator.button(4).onTrue(wrist.runOnce(() -> wrist.setWristPosition("bargePos")));
    operator.button(5).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L4Pos")));
    if (elevator.getHeight() == 1) {
      operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("algaeintake")));
    }
    else if (elevator.getHeight() == 2 || elevator.getHeight() == 3) {
      operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L23Pos")));
    }

    operator.button(8)
      .onTrue(algaeIntake.in())
      .onFalse(algaeIntake.stop());
    operator.button(7)
      .onTrue(algaeIntake.out())
      .onFalse(algaeIntake.stop());

    algaeIntake.stopOnAlgaeBinding();

    operator.button(2).onTrue(coralIntake.in());
    operator.button(1).onTrue(coralIntake.out());

    operator.axisLessThan(0, -.5).onTrue(coralSlide.goLeft());
    operator.axisGreaterThan(0, .5).onTrue(coralSlide.goCenter());
    operator.axisGreaterThan(1, .5).onTrue(coralSlide.goRight());

    operator.button(12).onTrue(elevator.runOnce(()->elevator.SetLevel(1)));
    operator.button(11).onTrue(elevator.runOnce(()->elevator.SetLevel(2)));
    operator.button(10).onTrue(elevator.runOnce(()->elevator.SetLevel(3)));
    operator.button(9).onTrue(elevator.runOnce(()->elevator.SetLevel(4)));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
