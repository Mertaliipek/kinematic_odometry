// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class KinematicCmd extends CommandBase {
  private final DriveTrain driveTrain;
  private final double xSpeed;
  private final double rot;

  
  /** Creates a new Kinematic. */
  public KinematicCmd(DriveTrain driveTrain,double xSpeed,double rot) {
    this.driveTrain = driveTrain;
    this.xSpeed = xSpeed;
    this.rot = rot;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(xSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
