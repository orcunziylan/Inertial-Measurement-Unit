/*
 * Demo sketch for new "PLX DAQ v2"
 * including most all common commands to be used
 */

int i = 0;

void setup() {

  // open serial connection
    Serial.begin(9600);

    //Serial.println("CLEARDATA"); // clears sheet starting at row 2
    Serial.println("CLEARSHEET"); // clears sheet starting at row 1
    
  // define 5 columns named "Date", "Time", "Timer", "Counter" and "millis"
    Serial.println("LABEL,Date,Time,Timer,Counter,millis");

  // set the names for the 3 checkboxes
    Serial.println("CUSTOMBOX1,LABEL,Stop logging at 250?");
    Serial.println("CUSTOMBOX2,LABEL,Resume log at 350?");
    Serial.println("CUSTOMBOX3,LABEL,Quit at 450?");

  // check 2 of the 3 checkboxes (first two to true, third to false)
    Serial.println("CUSTOMBOX1,SET,1");
    Serial.println("CUSTOMBOX2,SET,1");
    Serial.println("CUSTOMBOX3,SET,0");
}

void loop() {

    // simple print out of number and millis. Output e.g.,: "DATA,DATE,TIME,TIMER,4711,13374,AUTOSCROLL_20"
      //Serial.println( (String) "DATA,DATE,TIME,TIMER," + i++ + "," + millis() + ",AUTOSCROLL_20" );
      // alternative writing method:
         Serial.print("DATA,DATE,TIME,TIMER,");
        Serial.print(i++); Serial.print(",");
        Serial.println(millis()); 
        Serial.print(","); Serial.println(""); 

   
      
}
