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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.CoralSlide;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

public class RobotContainer {
  private final CommandXboxController driver, operator;
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final CoralIntake coralIntake = new CoralIntake();
  private final CoralSlide coralSlide = new CoralSlide();
  private final Wrist wrist = new Wrist(coralSlide.wristLimit());
  private final Elevator elevator = new Elevator(wrist.elevUpperLimit());
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  
  public RobotContainer() {
    driver = new CommandXboxController(1);
    operator = new CommandXboxController(0);
     
    configureBindings();

    drivetrain.setDefaultCommand(new RunCommand(
            () -> drivetrain.drive(
                circularScale(-MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.kDriveDeadband)),  
                circularScale(-MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.kDriveDeadband)),
                circularScale(-MathUtil.applyDeadband(driver.getRightX(), OperatorConstants.kDriveDeadband)),
                true, false), drivetrain)
        );
  /*   elevator.setDefaultCommand(new RunCommand( //TODO: take this out!!
      () -> elevator.drive(-MathUtil.applyDeadband(driver.getRightY(), OperatorConstants.kDriveDeadband)), elevator)
      ); */
  }

  private Trigger slideLeftTrigger = coralIntake.slideLeftLimit(),
  slideRightTrigger = coralIntake.slideRightLimit(),
  slideCenterTrigger = coralIntake.slideCentered();
  
  private void configureBindings() {
    driver.rightBumper().whileTrue(drivetrain.run(drivetrain :: setX));
        

    driver.y().onTrue(drivetrain.runOnce((drivetrain :: zeroHeading)));
        
    
    // Delayed response to Wrist command to collectCoral
    wrist.coralRequested().and(slideCenterTrigger)
    .onTrue(wrist.runOnce(() -> 
      wrist.setWristPosition("collectCoral")
      ));
    
    //  for when to allow the Slide and Elevator controls
    Trigger safeToSlide = 
      new Trigger(() -> wrist.getPosition() > OperatorConstants.safeToSlide),
        safeToFold = 
      new Trigger(() -> elevator.getHeight() < OperatorConstants.safeElevator);
      
    safeToSlide.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("safe slide", true)))
    .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("safe slide", false)));
    slideCenterTrigger.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("safe fold", true)))
    .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("safe fold", false)));
  
// Wrist commands:
    operator.button(3).onTrue(wrist.runOnce(() -> wrist.setWristPosition("collectCoral")));
    // .and(safeToFold)
      // .onTrue(coralSlide.goCenter()
      // .andThen(wrist.requestCoral())); //keeps from collectCoral until Slide centered
    operator.button(4)
    .onTrue(wrist.runOnce(() -> wrist.setWristPosition("bargePos")));
    operator.button(5).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L4Pos")));
     operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("algaeintake")));
      // operator.button(6).onTrue(wrist.runOnce(() -> wrist.setWristPosition("L23Pos")));

      // while (elevator.getHeight() == ElevatorConstants.Level1) {
      //   operator.button(6).onTrue(wrist.runOnce(()-> wrist.setWristPosition("algaeIntake")));
        
      // }

      // operator.button(6).onTrue(wrist.runOnce(() -> {
      //   if (elevator.getHeight() == ElevatorConstants.Level1) {
      //     wrist.setWristPosition("algaeIntake");
      //   } else if (elevator.getHeight() == ElevatorConstants.Level2 || elevator.getHeight() == ElevatorConstants.Level3) {
      //     wrist.setWristPosition("L23Pos");
      //   }
      // }));

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
      .onFalse(coralIntake.stopAndMaintain());
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
    operator.axisLessThan(1, -.5)
    .and(safeToSlide)
    .onTrue(elevator.runOnce(()->elevator.SetLevel(5)));
    operator.button(12)
    .and(safeToSlide)
    .onTrue(elevator.runOnce(()->elevator.SetLevel(1)));
    operator.button(11)
    .and(safeToSlide)
    .onTrue(elevator.runOnce(()->elevator.SetLevel(2)));
    operator.button(10)
    .and(safeToSlide)
    .onTrue(elevator.runOnce(()->elevator.SetLevel(3)));
    operator.button(9)
    .and(safeToSlide)
    .onTrue(elevator.runOnce(()->elevator.SetLevel(4)));
  }

  public Command getAutonomousCommand() {
    return drivetrain.run(() -> drivetrain.drive(0.25, 0, 0, false, false)).withTimeout(1.5);
    // return Commands.print("None configured");

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
