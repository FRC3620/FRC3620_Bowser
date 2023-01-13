// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.System.Logger.Level;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class autoLevelingCommand extends CommandBase {
  AHRS ahrs; 
  DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
  double roll;

  enum MyState {
    LEVEL, TILTED, LEVEL2, DONE
  } 
  
  MyState myState;

  /** Creates a new autoLevelingCommand. */
  public autoLevelingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = driveSubsystem.getRoll();

    if(myState == MyState.LEVEL){
      //drive
      if(roll > 10) {
        myState = MyState.TILTED;
      }
    } else if(myState == MyState.TILTED){
      //drive
     if(roll < 10){
       myState = MyState.LEVEL2;
      }
    }
    if(myState == MyState.LEVEL2){
      //stop
      myState = MyState.DONE;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean result = false;
    if(myState == MyState.DONE){
      result = true;
    }

    return result;
  }
}
