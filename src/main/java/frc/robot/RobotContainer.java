// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.CoralSlide;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final CommandXboxController driver, operator;
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final CoralIntake coralIntake = new CoralIntake();
  private final CoralSlide coralSlide = new CoralSlide();
  private final Wrist wrist = new Wrist(coralSlide.wristLimit());
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  
  public RobotContainer() {
    driver = new CommandXboxController(1);
    operator = new CommandXboxController(0);
     
    // TODO: add operator subsystems.
    configureBindings();

    drivetrain.setDefaultCommand(new RunCommand(
            () -> drivetrain.drive(
                circularScale(-MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.kDriveDeadband)),  
                circularScale(-MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.kDriveDeadband)),
                circularScale(-MathUtil.applyDeadband(driver.getRightX(), OperatorConstants.kDriveDeadband)),
                true, false), drivetrain)
        );
    elevator.setDefaultCommand(new RunCommand( //TODO: take this out!!
      () -> elevator.drive(-driver.getRightY()), elevator)
      );
  }

  private Trigger slideLeftTrigger = coralIntake.slideLeftLimit(),
  slideRightTrigger = coralIntake.slideRightLimit(),
  slideCenterTrigger = coralIntake.slideCentered();
  private boolean collectCoralRequest = false;

  private void configureBindings() {
    // Delayed response to Wrist command to collectCoral
    slideCenterTrigger
    .onTrue(wrist.runOnce(() -> {
      if(collectCoralRequest) wrist.setWristPosition("collectCoral");
      collectCoralRequest = false;
    }));
    
    //  for when to allow the Slide controls
    Trigger safeToSlide = 
      new Trigger(() -> wrist.getPosition() > OperatorConstants.safeToSlide);
      
    safeToSlide.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("safe slide", true)))
    .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("safe slide", false)));
    slideCenterTrigger.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("safe fold", true)))
    .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("safe fold", false)));
  
// Wrist commands:
    operator.button(3).and(slideCenterTrigger.negate())
      .onTrue(coralSlide.goCenter()
      .andThen(new InstantCommand(() -> collectCoralRequest = true))); //keeps from collectCoral until Slide centered
    operator.button(3).and(slideCenterTrigger)
      .onTrue(wrist.runOnce(() -> 
        wrist.setWristPosition("collectCoral")));
    operator.button(4)
    .onTrue(wrist.runOnce(() -> wrist.setWristPosition("bargePos"))
    .andThen(new InstantCommand(() -> collectCoralRequest = false)));
    operator.button(5).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L4Pos"))
    .andThen(new InstantCommand(() -> collectCoralRequest = false)));
    if (true/* elevator.getHeight() == 1 */) {
      operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("algaeintake"))
      .andThen(new InstantCommand(() -> collectCoralRequest = false)));
    }
    else if (elevator.getHeight() == 2 || elevator.getHeight() == 3) {
      operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L23Pos"))
      .andThen(new InstantCommand(() -> collectCoralRequest = false)));
    }

    //AlgaeIntake commands:
    operator.button(8)
      .onTrue(algaeIntake.in())
      .onFalse(algaeIntake.stop());
    operator.button(7)
      .onTrue(algaeIntake.out())
      .onFalse(algaeIntake.stop());

    algaeIntake.stopOnAlgaeBinding();

    //CoralIntake commands:
    coralIntake.stopOnCoralBinding();

    operator.button(2)
      .onTrue(coralIntake.in())
      .onFalse(coralIntake.stop());
    operator.button(1)
     .onTrue(coralIntake.out())
     .onFalse(coralIntake.stop());

     //Slide commands:
    operator.axisLessThan(0, -.5)
    .and(safeToSlide)
    .onTrue(coralSlide.goLeft());
    operator.axisGreaterThan(0, .5).onTrue(coralSlide.goCenter());
    operator.axisGreaterThan(1, .5)
    .and(safeToSlide)
    .onTrue(coralSlide.goRight());
    // response to the slide hitting a limit switch
    slideLeftTrigger.onTrue(coralSlide.resetLeft());
    slideRightTrigger.onTrue(coralSlide.resetRight());

  // response to elevator control buttons
    operator.button(12).onTrue(elevator.runOnce(()->elevator.SetLevel(1)));
    operator.button(11).onTrue(elevator.runOnce(()->elevator.SetLevel(2)));
    operator.button(10).onTrue(elevator.runOnce(()->elevator.SetLevel(3)));
    operator.button(9).onTrue(elevator.runOnce(()->elevator.SetLevel(4)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  
  double inputScale(double input, int scale) {
    boolean isNegative = input < 0;
    double out;
    if (isNegative) {
      out = -(1 - ((1 - DriveConstants.kMinSpeedMultiplier) * driver.getLeftTriggerAxis())) * (Math.pow(input, scale));
    }
    else {
      out = (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * driver.getLeftTriggerAxis())) * (Math.pow(input, scale));
    }
    return(out);
  }
  double circularScale(double in) {
  boolean isNegative = in < 0;
  double out;
  if (isNegative) {
    out = (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * driver.getLeftTriggerAxis())) * (Math.sqrt(1 - Math.pow(in, 2)) - 1);
  }
  else {
    out = (1 - ((1 - DriveConstants.kMinSpeedMultiplier) * driver.getLeftTriggerAxis())) * (-Math.sqrt(1 - Math.pow(in, 2)) + 1);
  }
  return(out);
}

}
