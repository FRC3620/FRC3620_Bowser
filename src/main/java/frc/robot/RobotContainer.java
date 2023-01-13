/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem;
  public static DriveSubsystem driveSubsystem;


  private final DriveCommand m_autoCommand = null;

  static public Joystick m_driverController = new Joystick(0);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //compressor.disable();
    DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 0, 1);

    Spark r0 = new Spark(0);
    Spark r1 = new Spark(1);
    MotorControllerGroup r = new MotorControllerGroup(r0, r1);
    r.setInverted(true);
    Spark l0 = new Spark(2);
    Spark l1 = new Spark(3);
    MotorControllerGroup l = new MotorControllerGroup(l0, l1);
    DifferentialDrive drive = new DifferentialDrive(l, r);

    m_driveSubsystem = new DriveSubsystem(drive, gearShiftSolenoid);
    //gearShiftSolenoid.setSubsystem("foo");
    //drive.setSubsystem("foo");

    m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, m_driverController));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new InstantCommand(() -> m_driveSubsystem.shiftToLowGear()));
    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_driveSubsystem.shiftToHighGear()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
