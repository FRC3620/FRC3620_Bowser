// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PitchAndRollSubsystem extends SubsystemBase {

  LinearFilter f_x = LinearFilter.movingAverage(50);
  LinearFilter f_y = LinearFilter.movingAverage(50);
  LinearFilter f_z = LinearFilter.movingAverage(50);

  double accel_center_x, accel_center_y, accel_center_z;
  Accelerometer accelerometer = new BuiltInAccelerometer();

  boolean amBaseLining = true;

  public void baseline(double x, double y, double z) {
    accel_center_x = f_x.calculate(x);
    accel_center_y = f_y.calculate(y);
    accel_center_z = f_z.calculate(z);
    SmartDashboard.putNumber("x_c", accel_center_x);
    SmartDashboard.putNumber("y_c", accel_center_y);
    SmartDashboard.putNumber("z_c", accel_center_z);

  }

  /**
   * https://www.hobbytronics.co.uk/accelerometer-info
   */
  public double getPitch(double x, double y, double z) {
    // Using x y and z from accelerometer, calculate x and y angles

    // Lets get the deviations from our baseline
    double x_val = x - accel_center_x;
    double y_val = y - accel_center_y;
    double z_val = z - accel_center_z;

    // need this no matter what
    double z2 = z_val * z_val;

    double y2 = y_val*y_val;
    double roll = Math.atan(x_val / Math.sqrt(y2 + z2));
    SmartDashboard.putNumber("roll", roll);
 
    double x2 = x_val * x_val;
    double pitch = Math.atan(y_val / Math.sqrt(x2 + z2));
    SmartDashboard.putNumber("pitch", pitch);

    return pitch;
  }

  @Override
  public void periodic() {
    double x = accelerometer.getX();
    double y = accelerometer.getY();
    double z = accelerometer.getZ();
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);

    if (amBaseLining) {
      baseline(x, y, z);
    } else {
      getPitch(x, y, z);
    }
  }

  public void stopBaselining() {
    amBaseLining = false;
  }

}
