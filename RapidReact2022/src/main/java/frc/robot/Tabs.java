package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Tabs {
    //Tabs
    public static ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
    public static ShuffleboardTab testingTab = Shuffleboard.getTab("Testing");
    public static ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");
    public static ShuffleboardTab graphTab = Shuffleboard.getTab("Graphs");
    public static ShuffleboardTab networkTab = Shuffleboard.getTab("Network Tables");

    //Entries
    //DRIVEBASE
    public static NetworkTableEntry angleEntry, pitchEntry, yawEntry, rollEntry, rotationsEntry, turnRateEntry;
    public static NetworkTableEntry leftSpeedEntry, rightSpeedEntry, speedEntry, leftPositionEntry, rightPositionEntry;
    public static NetworkTableEntry shifterEntry, shifterComp;
    public static NetworkTableEntry leftSpeedGraph, rightSpeedGraph, speedGraph, leftPositionGraph, rightPositionGraph, turnRateGraph;
    public static NetworkTableEntry axisTester;
}
