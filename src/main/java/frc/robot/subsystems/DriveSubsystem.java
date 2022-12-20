/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  enum WhichGear {
    UNKNOWN, LOW, HIGH
  }

  DifferentialDrive drive;

  WhichGear currentGear, desiredGear;
  DoubleSolenoid gearshiftSolenoid;
  Timer gearshiftTimer = new Timer();
  boolean gearshiftTimerIsActive = false;

  /**
   * Creates a new ExampleSubsystem
   */
  public DriveSubsystem(DifferentialDrive _drive, DoubleSolenoid _gearShiftSolenoid) {
    drive = _drive;
    gearshiftSolenoid = _gearShiftSolenoid;
    currentGear = WhichGear.UNKNOWN;
    desiredGear = WhichGear.LOW;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
      if (currentGear != desiredGear) {
        Value solenoidPosition = Value.kOff;
        if (desiredGear == WhichGear.LOW) {
          solenoidPosition = Value.kForward;
        } else {
          solenoidPosition = Value.kReverse;
        }
        gearshiftSolenoid.set(solenoidPosition);
        logger.info ("turned solenoid to " + solenoidPosition);
        currentGear = desiredGear;

        gearshiftTimer.reset();
        gearshiftTimer.start();
        gearshiftTimerIsActive = true;
      }
    }

    if (gearshiftTimerIsActive) {
      if (gearshiftTimer.hasElapsed(0.5)) {
        logger.info ("turned solenoid off");
        gearshiftTimer.stop();
        gearshiftSolenoid.set(Value.kOff);
        gearshiftTimerIsActive = false;
      }
    }
  }

  public void shiftToHighGear() {
    desiredGear = WhichGear.HIGH;
    logger.info ("Requested high gear");
  }

  public void shiftToLowGear() {
    desiredGear = WhichGear.LOW;
    logger.info ("Requested low gear");
  }

  public void driveRobot(double speed, double turn) {
    drive.arcadeDrive(speed, turn);
  }

  public void stop() {
    drive.stopMotor();
  }
}
