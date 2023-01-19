// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class stayLevelCommand extends CommandBase {
  AHRS ahrs; 
  private DriveSubsystem driveSubsystem;
  double roll;
  double speed;

  /** Creates a new stayLevelCommand. */
  public stayLevelCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    SmartDashboard.putBoolean("StayLevel Running", false);
    SmartDashboard.putNumber("StayLevel Speed", speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("StayLevel Running", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = driveSubsystem.getRoll();

    if(roll > 13){
      roll = 13;
    } else if(roll < -13){
      roll = -13;
    }

    if(roll > 0){
      speed = -((roll / 18));
    } else {
      speed = -((roll / 18));
    }

    SmartDashboard.putNumber("StayLevel Speed", speed);
    driveSubsystem.driveRobot(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    SmartDashboard.putBoolean("StayLevel Running", false);
    speed = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
