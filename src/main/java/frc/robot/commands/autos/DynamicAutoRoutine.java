// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;


/** Add your docs here. */
public class DynamicAutoRoutine extends SequentialCommandGroup {

    String m_instruction = "";
    Drive m_drive;

/* 
public DynamicAutoRoutine(Drive a_drive){
    // Constructor
    m_drive = a_drive;
    //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
    m_instruction = "";
}
*/

public DynamicAutoRoutine(Drive a_drive){
    // Constructor
    m_drive = a_drive;
    //Populate some string from Dashboard with format <ReefPosition><ReefLevel>..." ( ie - A1B2C3D4 ; A4B4C4D4 ; etc )
    m_instruction = SmartDashboard.getString("DynamicAutoInput", "");

    if (m_instruction == null || m_instruction.isEmpty()) {

        System.out.println("request is empty or unexpected... instruction = " + m_instruction);
        m_instruction = "";
    }

    List<Pair<String, Integer>> parsedInstructions = parseInstruction(m_instruction);
    // TODO : Break down instruction

    for (Pair<String, Integer> pair : parsedInstructions) {
        String position = pair.getFirst();
        int level = pair.getSecond();
        // Add commands based on position and level
        addCommands(
            m_drive.pathfindThenFollowPath(DriveConstants.pathingConstraints,"goto_" + position),
            new WaitCommand(1),// TODO : scoreAtLevel(level)
            m_drive.pathfindThenFollowPath(DriveConstants.pathingConstraints,"goto_" + "l"+ "_" + "left"),
            new WaitCommand(1) // TODO : collectFromCoralStation()
        );
    }
    
    
    // addCommands(
    //     // From Start Line to Reef
    //     m_drive.pathfindThenFollowPath(DriveConstants.pathingConstraints,"Right"),
    //     // Score at the specified Level
    //     new WaitCommand(1),
    //     m_drive.pathFindToPose(DriveConstants.pathingConstraints, Constants.TargetLocations.ORIGIN),
    //     // From Reef to Coral Station
    //     new WaitCommand(1),
    //     // From Coral Station to Reef
    //     new WaitCommand(1)

    // ); 

}

private List<Pair<String, Integer>> parseInstruction(String instruction) {
    List<Pair<String, Integer>> parsedInstructions = new ArrayList<>();
    for (int i = 0; i < instruction.length(); i += 2) {
        String letter = instruction.substring(i, i + 1);
        int number = Integer.parseInt(instruction.substring(i + 1, i + 2));
        parsedInstructions.add(new Pair<>(letter, number));
    }
    return parsedInstructions;
}


}


