//YA RABI YA7FTH EROBOT
#define LeftMotor1 3// forward
#define LeftMotor2 6// backward
#define RightMotor1 10// forward
#define RightMotor2 11// backward
#define MotorCoeff 1
#define Sensor1 7
#define Sensor2 4
#define Sensor3 A4
#define Sensor4 2
#define Sensor5 A5
#define Sensor6 8
#define Sensor7 12
int flag=1;
float Kp=0.35;
float Ki=0.35;
float Kd=0.35;
int Sense = 0; //Respresnts the current direction of the robot
float lasterror=0;//variable to calculate derivate for pid
float error;
int penderations[7]={-3,-2,-1,0,1,2,3};//weights for each sensor to calculate the error for PID (normal PID mech el PID elli fi mokh 7amma XD)
float MatrixChoice = 0;  /*hna bdina nodkhlou fe tab3a matrixchoice variable helps us wich
                    weights matrix to use because in this code it depends on the part of the map.*/

float penderationsMatrix[5][7] = {          /*this weights matrix is used for parts of the map where we don't need
                                    to prioritiese turning neither to the right or the left.you can see that the wieghts are symetric.*/
  {-400,-145,-55,0,55,145,400},
  {-250,-75 ,-25,0, 25,75 ,250},
  {-100 ,-25 ,-5,0, 5,25 ,100},
  {-50 ,-10 , 0,0, 0 ,10 ,50},
  {-17 ,-5  , 0 ,0, 0, 5  ,17}
};



float ExtremeRightPenderationsMatrix[5][7] = {       /*this weights matrix is used for parts of the map where we  need
                                    to prioritiese turning  to the right .you can see that the wieghts in the rights are greater (in abs value) than those on the left.*/ 
  {-100,-55,-55,0,55,145,400},
  {-50,-25 ,-25,0, 25,75 ,250},
  {-25 ,-15 ,-5,0, 5,25 ,100},
  {-15 ,-5 , 0,0, 0 ,10 ,50},
  {-10 ,0  , 0 ,0, 0, 5  ,17}
};
float ExtremeLeftPenderationsMatrix[5][7] = {          /*this weights matrix is used for parts of the map where we  need
                                    to prioritiese turning  to the left .you can see that the wieghts in the left are greater (in abs value) than those on the right.*/

  {-400,-145,-55,0,55,55,100},
  {-250,-75 ,-25,0, 25,25 ,50},
  {-100 ,-25 ,-5,0, 5,15 ,25},
  {-50 ,-10 , 0,0, 0 ,5 ,15},
  {-17 ,5  , 0 ,0, 0, 0  ,10}
};


char sensorValues[7]; //sensor values in a string
bool* sensorValueMatrix[5]; //this is the matrix where we stock the last 5 sensor values to calculate the error for the matrix PID.


void setup() {
  // put your setup code here, to run once:
  for(int i = 0; i < 5; i++){            /*in the beginning we fill our matrix as if our robot isin the write position and soes not need any correction*/
    sensorValueMatrix[i] = (bool*)malloc(sizeof(bool)*7);//this line is to create the columns of euch line.
    sensorValueMatrix[i][0] = 0;
    sensorValueMatrix[i][1] = 0;
    sensorValueMatrix[i][2] = 0;
    sensorValueMatrix[i][3] = 1;
    sensorValueMatrix[i][4] = 0;
    sensorValueMatrix[i][5] = 0;
    sensorValueMatrix[i][6] = 0;
  }
  pinMode(LeftMotor1,OUTPUT);
  pinMode(LeftMotor2,OUTPUT);
  pinMode(RightMotor1,OUTPUT);
  pinMode(RightMotor2,OUTPUT);
  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);
  pinMode(Sensor6, INPUT);
  pinMode(Sensor7, INPUT);
  Serial.begin(9600);
  runMotors(100,100);//these 2 lines are to make the robot get out of the starting square(we've faced a problem here doing it in a seperate case so we did it in the dumbest way possible)
  delay(500);
}


void Detectflag(){
  switch(flag){
    case 1:
      if((sensorValues[1]=='1'||sensorValues[0]=='1')&&(sensorValues[5]=='1'||sensorValues[6]=='1')){ //in these conditions the robot passes to the next flag(part) where he priorities turning to the right. 
        flag +=1;
        MatrixChoice = 1;//we choosed the matrix the priorities turning to the right 
        
        }
      break;
    case 2:
    if(!strcmp(sensorValues,"0000000")){//in these conditions the robot passes to the next flag(part) where it follows the line using the pid and it does not prioritese neither turning to the left or the right.
      flag +=1;
      MatrixChoice = 0;
    }
    break;
    case 3:
    if(sensorValues[0]=='1'&&sensorValues[6]=='1'){//in these conditions the robot passes to the next flag(part).
        flag +=1;
        }
      break;
    case 4://konna najmou ma nektbouhech star hetha khater passage lel flag elli ba3dou sar fel loop
      break;
    case 5:
      MatrixChoice = 1; //priority for turning to the right in this part of the map
      if(!strcmp(sensorValues,"0000000")){//in these conditions the robot passes to the next flag(part)
      flag +=1;
    }
    break;
    case 6:
    if((sensorValues[1]=='1'||sensorValues[0]=='1')&&(sensorValues[5]=='1'||sensorValues[6]=='1')){//in these conditions the robot passes to the next flag(part)
        flag =7;
        MatrixChoice = 0;//we do not priorities turning to neither the right or the left in this part of the map
        }
    break;
    case 7:
       if((sensorValues[1]=='1')&&(sensorValues[5]=='1')){//in these conditions the robot passes to the next flag(part)
        flag =8;
        }
        break;
    
      
    default:
      Serial.print("no way");
      break;
  }
}
int ZigZagSpeed = 180;//initialy the zizag part speed is set at 180
void loop() {
  if(flag!=7){
    ReadSensors(10,10);//we read the sensors normaly when we are out of the flag(part) 7 
  }else{
    ReadSensorsInverse(10,10);//we read it inverted in the flag(part) 7 because the robot is now following the white line.
  }
  
  Detectflag();//we detect in there is a flag (a condition satisfied to pass to the next part of the map)
  switch(flag){//we go to the part of the map where we are operating
    case 1:
      RunPIDMatrix(220);//the first part is simple( not very simple for me but simple for the one who code it)PID
      break;
    case 2:
 
      RightFilterExtreme();//besides prioritising turning to the right in the matrix we've made a function to make the left sensorsgo blind if the matrix alone wasn't enogh
      RunPIDMatrix(220);//PID 220 speed
      break;
    case 3:
      
      No9taFilter(750,450);// a function to let some sensors on both the left and the right go blind for a defined period of time to avoid following the the small no9ta on the side of the main line
      RunPIDMatrix(220);
      break;
    case 4:
      runForward(450,170);//robot runs forward for 450 milliseconds 170 speed we didthis because the pid when ith sees all black it does not function properly
      runForward(5000,0);//robot stops for 5 seconds
      runForward(450,180);//robot runs forward for 450 millisecond 180 speed 
      flag += 1;// we move to the next part of the map 
      break;
    case 5:
      RightFilterExtreme();//besides prioritising turning to the right in the matrix we've made a function to make the left sensorsgo blind if the matrix alone wasn't enogh
      RunPIDMatrix(150);// PID 150 speed
    break;
    case 6:

      RunPIDMatrix(180); //simple PID
      break;
    case 7:
      RunPIDMatrix(180);//simple PID with inverted censors values because in this part the robot is following the whiite line
 
    break;
    case 8:
   
    ZigZagFilter(250);//Zigzag part when you get to the end of a line and you need to take a big turn to the right or the left the robot stops make some changes(we begin to use different weights matrix and filter some sensors) we make also some changes on the speed these changes are valid for 250 milliseconds for each big turn and then the robot runs a normal pid
    RunPIDMatrix(ZigZagSpeed);
   
      break;
    case 9:
      runForward(400, 150);//robot runs forward for 400 milliseconds 150 speed
      runMotors(0,0);//robot stops in final case
      flag = 98;// the end (there is no case 98)
      break;
      case 99:
      Serial.println(sensorValues); //case for tests
      delay(500);
      break;
    default:
      runMotors(0,0);
      break;
  }
}

void followLineBasic(){// we didn't use this function but you can check it we also used the variable sense here to let the robot remember the last move he was doing if all the sensors got out of the line
  if(sensorValues[6]=='1')
    {runMotors(20,100);Sense =-1;}
  else if(sensorValues[0]=='1')
    {runMotors(100,20);Sense =1;}
  else if(!strcmp(sensorValues,"01000"))
    {runMotors(120,50);Sense =1;}
  else if(!strcmp(sensorValues,"00010"))
    {runMotors(50,120);Sense =-1;}
  else if(!strcmp(sensorValues,"01100"))
    {runMotors(60,120);Sense =1;}
  else if(!strcmp(sensorValues,"00110"))
    {runMotors(120,60);Sense =-1;}
  else if(!strcmp(sensorValues,"00100"))
    {runMotors(200,200);Sense =0;}
  else if(!strcmp(sensorValues,"00000"))
    {
      if(Sense == 1){
        
        runMotors(100,-50);
      }else if(Sense == -1){
        runMotors(-50,100);
      }
    }
}





long no9taTimerStart = 0;
bool no9taStarted = false;
void No9taFilter(int Duration, int awaiting){
  if(sensorValues[6]=='1'&&sensorValues[5]=='1'&&!no9taStarted){//in this case the no9tafiltrewill begin to work (when the robot gets to the 90° turn and the no9tafiltre does not all ready work )
    no9taStarted = true; //this indicates that no9tafilter is now working
    no9taTimerStart = millis();//millis() returns for how many milliseconds the arduino was working (since reset or since activating it withe interuupter)
  }
  if((millis()-no9taTimerStart>awaiting)&&(millis()-no9taTimerStart<(Duration+awaiting))&&no9taStarted){//no9ta filter practicly won't work only after a certain time wich is the awaiting and it will work for determined period of time wich is theduration
    sensorValueMatrix[0][0]=0;//with these 2 lines the robot is able to ignore the no9ta that can be on the left or the right
    sensorValueMatrix[0][6]=0;
  }
}

long timer;
float LeftDominant = 0;
int counter = 0;
void ZigZagFilter(int duration){//with this function the robot is able to make a stop at the edge of every line of the zizag and make sansors filtration and change pid weights matrix and speed to make the big turn that must do 
  if(LeftDominant == 0){//in this case we are not prioritising turning to the left or the right
    if(!strcmp(sensorValues,"1111111")||!strcmp(sensorValues,"1111100")){//here the robot reached a zigzag line edge he must stop 
      ZigZagSpeed = 150;//directly after the stop the robot speed will be 150
      runMotors(0, 0);//robot stop for 100 milliseconds
      delay(100);
      Ki= 0.4;//we chaged pid constants to provide a smooth functioning
      Kp = 0.15;
      Kd = 0.15;
      timer = millis();
      if(counter==3){flag++;}//we are going to make 4 big turns an then pass to the final flag(part of the map)
      if(counter==0||counter==2){//the first and the third turns are big turns to the left
        LeftDominant= -1;
      }else{//the rest of the turns are big turns to the right
        LeftDominant = 1;
      }
      counter++;//we have made a big turn now we are waiting for the next one
    }
  }
  if((millis()-timer<duration)){//for th specified period of time some sensors will get filtred and the weights matrix will be changed so that the robot could make the turn to the left or the right
    if(LeftDominant == 1){//we need to make a big turn to the left
      if(sensorValues[0]=='1'||sensorValues[1]=='1'){
      sensorValueMatrix[0][4] = 0;
      sensorValueMatrix[0][5] = 0;
      }
      sensorValueMatrix[0][6] = 0;

      MatrixChoice = 2;
    }else if(LeftDominant == -1){//we need to make a big turn to the right
      if(sensorValues[6]=='1'||sensorValues[5]=='1'){
      sensorValueMatrix[0][2] = 0;
      sensorValueMatrix[0][1] = 0;
      }
      sensorValueMatrix[0][0] = 0;
      MatrixChoice = 1;
    }
  }else if(LeftDominant!=0) {//the duration specified is passed and there is no need to make big  turns for the moment so the robot returns to normal functioning
    ZigZagSpeed = 180;
    LeftDominant=0;
    Kp = 0.45;
    Kd = 0.45;
    MatrixChoice = 0;
  }
}



void RightFilterExtreme(){//filter to let the robot think he needs to prioritise turning to the right
  if((sensorValues[4]=='1'||sensorValues[5]=='1'||sensorValues[6]=='1')){
    sensorValueMatrix[0][0]=='0';
    sensorValueMatrix[0][1]=='0';
    sensorValueMatrix[0][2]=='0';
    sensorValueMatrix[0][3]=='0';
  }
}

void runMotors(int pwmRight, int pwmLeft){//normal runmotors function
  pwmLeft *= MotorCoeff;
  if (pwmLeft>=0){
    analogWrite(LeftMotor1, pwmLeft);
    analogWrite(LeftMotor2, 0);

  }else{
    analogWrite(LeftMotor1, 0);
    analogWrite(LeftMotor2,pwmLeft );

  }
  if (pwmRight>=0){
    analogWrite(RightMotor1, pwmRight);
    analogWrite(RightMotor2, 0);

  }else{
    analogWrite(RightMotor1, 0);
    analogWrite(RightMotor2,pwmRight );

  }
  }


  int currentSensorValues[7];
  void ReadSensors(int desiredDelay, int samplesPerDelay){//function to read censors and filter noise (making an average of a certain number of sensor values took in determined period of time)
    int calculatedDelay = desiredDelay/samplesPerDelay;
    for(int i = 0; i < 7; i++){//emptying current sensor values to calculate the average in it
      currentSensorValues[i]=0;
      }
    for(int i = 0; i < samplesPerDelay; i ++){
      currentSensorValues[0] += CheckSensor(digitalRead(Sensor1));//calculating the average given by each sensor(cecksensor returns 1 or -1 if values of sensor =0 are more than those =1 then currentsensorvalues[i] is negative)
      currentSensorValues[1] += CheckSensor(digitalRead(Sensor2));
      currentSensorValues[2] += CheckSensor(digitalRead(Sensor3));
      currentSensorValues[3] += CheckSensor(digitalRead(Sensor4));
      currentSensorValues[4] += CheckSensor(digitalRead(Sensor5));
      currentSensorValues[5] += CheckSensor(digitalRead(Sensor6));
      currentSensorValues[6] += CheckSensor(digitalRead(Sensor7));
      delay(calculatedDelay);//for example we are going to take 5 samples in 25 milliseconds so the delay between each sample is 5 millisecond
    }
    sprintf(sensorValues, "%d%d%d%d%d%d%d", currentSensorValues[0]>=0,//if the currentsensorvalue[i]<0 then the samples where we found sensor i =0 are more than those where we found sensor i =1 so we are going take the value of sensor i =0
                                        currentSensorValues[1]>=0,
                                        currentSensorValues[2]>=0,
                                        currentSensorValues[3]>=0,
                                        currentSensorValues[4]>=0,
                                        currentSensorValues[5]>=0,
                                        currentSensorValues[6]>=0
                                        );
                                        
    bool* inter = sensorValueMatrix[4];//a variable to stock last line of the matrix because every reading we are destroing the last line of the matrix
    for(int i = 4; i > 0; i--){
      sensorValueMatrix[i] = sensorValueMatrix[i-1];//decalage des ligne de la matrice(if we don't write the line commented above there is a line tha is going to be lost for more information contact me )
    }
    sensorValueMatrix[0] = inter;//now we put the last line of the matrix in the first line to replace it
    for(int i = 0; i < 7;i++){//we put our new reading in the first line
      sensorValueMatrix[0][i]= currentSensorValues[i]>=0;
    }

  }
  void ReadSensorsInverse(int desiredDelay, int samplesPerDelay){//same as read censors but inverted
    int calculatedDelay = desiredDelay/samplesPerDelay;
    for(int i = 0; i < 7; i++){
      currentSensorValues[i]=0;
      }
    for(int i = 0; i < samplesPerDelay; i ++){
      currentSensorValues[0] += CheckSensor(digitalRead(Sensor1));
      currentSensorValues[1] += CheckSensor(digitalRead(Sensor2));
      currentSensorValues[2] += CheckSensor(digitalRead(Sensor3));
      currentSensorValues[3] += CheckSensor(digitalRead(Sensor4));
      currentSensorValues[4] += CheckSensor(digitalRead(Sensor5));
      currentSensorValues[5] += CheckSensor(digitalRead(Sensor6));
      currentSensorValues[6] += CheckSensor(digitalRead(Sensor7));
      delay(calculatedDelay);
    }
    sprintf(sensorValues, "%d%d%d%d%d%d%d", currentSensorValues[0]<=0,
                                        currentSensorValues[1]<=0,
                                        currentSensorValues[2]<=0,
                                        currentSensorValues[3]<=0,
                                        currentSensorValues[4]<=0,
                                        currentSensorValues[5]<=0,
                                        currentSensorValues[6]<=0
                                        );
                                        
    bool* inter = sensorValueMatrix[4];
    for(int i = 4; i > 0; i--){
      sensorValueMatrix[i] = sensorValueMatrix[i-1];
    }
    sensorValueMatrix[0] = inter;
    for(int i = 0; i < 7;i++){
      sensorValueMatrix[0][i]= currentSensorValues[i]<=0;
    }
    
  }
  int CheckSensor(bool value){
    if(value)
      return 1;
    return -1;
  }
  float claculatError(){//calculate error for normal pid we didn't use it
    float error=0;
    for(int i=0;i<7;i++){
      error+=(sensorValues[i]=='1')*penderations[i];
    }
    return error;

  }
  float calculateErrorMatrix(){//calculate error for matrix pid and chosing the weights matrix depends on the variable matrixchoice
    float error=0;
    for(int i=0;i<5;i++){
      for(int j = 0; j < 7; j++){
        if(MatrixChoice==0){
        error+=(sensorValueMatrix[i][j])*penderationsMatrix[i][j];
        }else if(MatrixChoice==1){
        error+=(sensorValueMatrix[i][j])*ExtremeRightPenderationsMatrix[i][j];
        }else{
        error+=(sensorValueMatrix[i][j])*ExtremeLeftPenderationsMatrix[i][j];
        }
      }
    }
    return error;
  }
  void RunPID(int speedMoy){//running normal pid
    int consigne = PID(claculatError());
    runMotors(constrict(speedMoy-consigne), constrict(speedMoy+consigne));
  }
  void RunPIDMatrix(int speedMoy){//running matrix pid
    int consigne = PID(calculateErrorMatrix());
    runMotors(constrict(speedMoy-consigne), constrict(speedMoy+consigne));//function constrict to limit the correction if it make the speed goes past 255 or -255 (pwm limit) 
  }
  float integral;
  int PID(float erreur){//function returns correction
    float output = 0;
    output=Kp*erreur;
    output+= Kd*(erreur-lasterror);
    output+= Ki*integral*(sensorValues[3] == '0'||sensorValues[2] == '0'||sensorValues[4] == '0');//the accumulated error is corrected only in consideration in this condition
    lasterror=erreur;
    integral += erreur;
    if(sensorValues[3] == '1'||sensorValues[2] == '1'||sensorValues[4] == '1'){integral=0;}//in this condition we are not accumulating error
    return output;
  }

  int constrict(int val){//function constrict to limit the correction if it make the speed goes past 255 or -255 (pwm limit)
    if(val>255){
      return 255;
    }
    if(val<-255){
      return -255;
    }
    return val;
  }
  
void showMatrix(){//fonction to check matrix values if needed in tests
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 7;j++){
      Serial.print(sensorValueMatrix[i][j]);
      Serial.print("|");
    }
    Serial.println("");
  }
  Serial.println("--------------------");
}
void runForward(int forwardDelay, int speed){//a function to make the robot run forward for a determined period of time in a certain speed.
  long startTime = millis();
  while(millis()-startTime< forwardDelay){
    runMotors(speed, speed);
  }
}