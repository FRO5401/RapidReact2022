package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Tabs.*;

public class NetworkTables extends SubsystemBase {
  
  NetworkTable visionTable, targetTable, robotTable;
  NetworkTableInstance inst;
  NetworkTableEntry ballXEntry, ballYEntry, ballDEntry, ballREntry;
  NetworkTableEntry targetXEntry, targetYEntry, targetDEntry;
  NetworkTableEntry robotXEntry, robotYEntry, robotDEntry;
  NetworkTableEntry shooterVModeEntry;
  private static double ballX, ballY, ballDistance;
  public double ballRadius;
  public int mode;
  public double targetX, targetY, targetDistance;
  public double robotX, robotY, robotDistance;


  public NetworkTables() {
    inst = NetworkTableInstance.getDefault();
    visionTable = inst.getTable("VisionTable");
    //targetTable = inst.getTable("Targets");
    //robotTable = inst.getTable("Robot");
    ballXEntry = visionTable.getEntry("ballcX");
    ballYEntry = visionTable.getEntry("ballcY");
    targetXEntry = visionTable.getEntry("targetcX");
    targetYEntry = visionTable.getEntry("targetcY");
    robotXEntry = visionTable.getEntry("robotX");
    robotYEntry = visionTable.getEntry("robotY");
    ballDEntry = visionTable.getEntry("ballDistance");
    targetDEntry = visionTable.getEntry("targetDistance");
    robotDEntry = visionTable.getEntry("robotDistance");
    ballREntry = visionTable.getEntry("ballRadius");
    shooterVModeEntry = visionTable.getEntry("mode");

    
    ballX = 0.0;
    ballY = 0.0;
    targetX = 0.0;
    targetY = 0.0;
    robotX = 0.0;
    robotY = 0.0;
    targetDistance = 0.0;
    robotDistance = 0.0;
    ballDistance = 0.0;
    ballRadius = 0.0;
    mode = 3;

    inst.startClientTeam(5401); // where TEAM=190, 294, etc, or use inst.
    inst.startDSClient();

    networkTablesShuffleboard();
  }

  @Override
  public void periodic() {
    updateValue();
    reportValues();
    //odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void updateValue() {
     // recommended if running on DS computer; this gets the robot

    ballX = ballXEntry.getDouble((ballX != 0) ? ballX : 0);
    ballY = ballYEntry.getDouble((ballY != 0) ? ballY : 0);
    ballRadius = (ballX == 0 && ballY == 0) ? 0 : ballREntry.getDouble((ballRadius != 0) ? ballRadius : 0);
    ballDistance = ballDEntry.getDouble((ballDistance != 0) ? ballDistance : 0);
    targetX = targetXEntry.getDouble((targetX != 0) ? targetX : 0);
    targetY = targetYEntry.getDouble((targetY != 0) ? targetY : 0);
    targetDistance = ballDEntry.getDouble((targetDistance != 0) ? targetDistance : 0);
    robotX = robotXEntry.getDouble((robotX != 0) ? robotX : 0);
    robotY = robotYEntry.getDouble((robotY != 0) ? robotY : 0);
    robotDistance = robotDEntry.getDouble((robotDistance != 0) ? robotDistance : 0);
    //mode = (int)shooterVModeEntry.getDouble(mode);
    //powerPortX = powerPortXEntry.getDouble(0.0);
    //powerPortY = powerPortYEntry.getDouble(0.0);
    //System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    //System.out.println("The Ball is " + ballDistance + " away");
    //System.out.println("The radius is " + radius);
    //System.out.println("The Power Port coordinates are: " + "X: " + powerPortY + " Y: " + powerPortY);
  }

  public double getBallXValue() {
    return ballX;
  }

  public double getBallYValue() {
    return ballY;
  }

  public double getBallRadius(){
    return ballRadius;
  }

  public double getBallDistance(){
    return ballDistance;
  }

  public double getTargetXValue() {
    return targetX;
  }

  public double getTargetYValue(){
    return targetY;
  }

  public double getRobotXValue() {
    return robotX;
  }

  public double getRobotYValue() {
    return robotY;
  }

  public double getTargetDistance(){
    return targetDistance;
  }

  public double getRobotDistance() {
    return robotDistance;
  }

  //1 is red, 2 is blue, 3 is shoot
  public void setMode(int mode) {
    shooterVModeEntry.setDouble(mode);
  }

  public int getMode() {
    return mode;
  }
  
  public void resetValues(){
    ballX = 0;
    ballY = 0;
    targetY = 0;
    targetX = 0;
    robotX = 0;
    robotY = 0;
    robotDistance = 0;
    targetDistance = 0;
    ballDistance = 0;
    ballRadius = 0;
  }

  public boolean checkCentered(){

    if(getBallXValue() >= 300 && getBallXValue() <= 340){
      return true;
    }
    
    else if(((getBallXValue() < 300) & (getBallXValue() >= 0)) || ((getBallXValue() > 340) & (getBallXValue() <= 640))){
      return false;
    }
    return false;
  }



  public void reportValues()
  {
    ballXShuffleboard.setDouble(getBallXValue());
    ballYShuffleboard.setDouble(getBallYValue());
    ballDShuffleboard.setDouble(getBallDistance());
    ballRShuffleboard.setDouble(getBallRadius());
    targetXShuffleboard.setDouble(getTargetXValue());
    targetYShuffleboard.setDouble(getTargetYValue());
    targetDShuffleboard.setDouble(getTargetDistance());
    robotXShuffleboard.setDouble(getRobotXValue());
    robotYShuffleboard.setDouble(getRobotYValue());
    robotDShuffleboard.setDouble(getRobotDistance());
    shooterVModeShuffleboard.setNumber(getMode());
  }

  public void networkTablesShuffleboard() {
    //Network config
    ballXShuffleboard = networkTab.add("Ball CX", getBallXValue()).getEntry();  
    ballYShuffleboard = networkTab.add("Ball CY", getBallYValue()).getEntry();  
    ballDShuffleboard = networkTab.add("Ball Distance", getBallDistance()).getEntry();  
    ballRShuffleboard = networkTab.add("Ball Radius", getBallRadius()).getEntry();  
    targetXShuffleboard = networkTab.add("Target CX", getTargetXValue()).getEntry();  
    targetYShuffleboard = networkTab.add("Target CY", getTargetYValue()).getEntry();  
    targetDShuffleboard = networkTab.add("Target Distance", getTargetDistance()).getEntry(); 
    robotXShuffleboard = networkTab.add("Robot CX", getRobotXValue()).getEntry();  
    robotYShuffleboard = networkTab.add("Robot CY", getRobotYValue()).getEntry();  
    robotDShuffleboard = networkTab.add("Robot Distance", getRobotDistance()).getEntry();  
    shooterVModeShuffleboard = networkTab.add("Mode", getMode()).getEntry();
    
  }
}