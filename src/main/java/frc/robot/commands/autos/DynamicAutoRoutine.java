// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

/** Add your docs here. */
public class DynamicAutoRoutine {

    String m_instruction = "";
    Drive m_drive = new Drive(null);


public DynamicAutoRoutine(Drive a_drive){
    // Constructor
    m_drive = a_drive;
    //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
    m_instruction = "";
}

public DynamicAutoRoutine(Drive a_drive, String request){
    // Constructor
    m_drive = a_drive;
    //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
    m_instruction = request;
}

public void execute()
{
    // TODO : Break down instruction

    // NOT DONE YET... THE INSTRUCTION NEEDS TO BE BROKEN DOWN INTO INDIVIDUAL PATHS, FOR THE APPROPRIATE STRETCHES 
    // IE - (FROM START -> REEF ; FROM REEF -> CORAL STATION ; FROM CORAL STATION -> REEF)
    m_drive.followPathCommand(m_instruction); 
}

}


