// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooserSubsystem extends SubsystemBase {
  /** Creates a new AutoChooserSubsystem. */
  SendableChooser<String> auto = new SendableChooser<String>();


  public AutoChooserSubsystem() 
  {
    auto.addOption("Position 1", "Position 1");
    auto.addOption("Position 2", "Position 2");
    auto.addOption("Position 3", "Position 3");
    auto.addOption("Position 4", "Position 4");
    auto.setDefaultOption("Default taxi", "Default taxi");

    SmartDashboard.putData(auto);
  }

  public String getAutoPath()
  {
    return auto.getSelected();
   // return SmartDashboard.getString("Auto Value", auto.getSelected());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getAutoPath();
    SmartDashboard.putString("Auto Value", auto.getSelected());


  }
}
