package frc.robot.Utilities;

//I am a genius
public class Printer {
    
    public static String genericList[] = {
        "Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel",
        "India", "Juliet", "Kilo", "Lima", "Mike", "November", "Oscar", "Papa", 
        "Quebec", "Romeo", "Sierra", "Tango", "Uniform", "Victor", "Whiskey", "Yankee", "Zulu"
    };

    //Print quicker
    public static void print(String printed){
        System.out.println(printed);
    }

    //qp means quick print
    public static void qp(int rapid){
        int index = (rapid>25) ? indexer(rapid) : rapid;
        System.out.println(genericList[index-1] + incrementer(rapid));
    }

    //Wasting
    private static int indexer(int rapid){
        while(rapid > 25){
            rapid-= 25;
        }
        return rapid;
    }

    //Time
    private static int incrementer(int rapid) {
        int counter = 0;
        while(rapid>25){
            rapid-= 25;
            counter++;
        }
        return counter;
    }

}
