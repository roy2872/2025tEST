// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controllers.controllers.GenericController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private Autonomous autonomous;
  private SwerveSubsystem swerve;
  private Field2d field;
  private GenericController controller;
    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher rotPub;
  public RobotContainer() {
    controller = new GenericController(0);
    autonomous = new Autonomous();
    swerve = new SwerveSubsystem();
    field = new Field2d();
    SmartDashboard.putData("field", field);

    swerve.setDefaultCommand(swerve.driveCommand(
    controller.getLeftX(),
    controller.getLeftY(),
    controller.getRightX(),
    () -> true )
  );

  
    
    configureBindings();
    

  }

  private void configureBindings() {

    // controller.getA().onTrue(new RunCommand(() -> swerve.rotateRobot(5)) );
  }

  public Command getAutonomousCommand() {
    return autonomous.getSelected();
  }
  
  public void periodic(){
    // System.out.println(controller.getRightX().getAsDouble());

  }
}
