#include <Adafruit_NeoPixel.h>
#define PIN 6
#define NUM 40
#define BUFFS 1024
#define DELEY 2000000

// strip controller
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

int deley=DELEY;

uint32_t red = strip.Color(80,0,0);

void set_color(double rad = 0,double d =0){
  
  //set default
  for(int i=0;i<NUM;i++)
    strip.setPixelColor(i, red);
    
  strip.show();
}

char* read_cmd(){
  char cmd[BUFFS];
  memset(cmd,0,BUFFS);
  int ind=0;
  cmd[0]='\0';
  while(true){
    if(Serial.available()>0){
      cmd[ind] = Serial.read();
      
      if(cmd[ind] == '\n' || cmd[ind]=='\r'){
        cmd[ind] = '\0';
        break;
      }
      
      cmd[++ind] = '\0';
    }
    
    //deley for close fire
    if(deley==1){
      //shutdown all
      deley--;
      for(int i=0;i<=40;i++){
        strip.setPixelColor(i,0,0,0);
      }
      strip.show();
    }else if(deley!=0)
      deley--;
  }
  return cmd;
}

const char sep[6] = "[], \n";
void decode_and_set(char* cmd){
  
  Serial.print(cmd);
  Serial.print("\0\n");
      
  char* token;
  int x,y;
      
  //get x,y
  token = strtok(cmd,sep); 
  x = atoi(token);
    
  token = strtok(NULL,sep);
  y = atoi(token);
  
  int r,g,b,a;
    
  //get r,g,b and alpha
  token = strtok(NULL,sep);
  r = atoi(token);
  
  token = strtok(NULL,sep);
  g = atoi(token);
  
  token = strtok(NULL,sep);
  b = atoi(token);
  
  token = strtok(NULL,sep);
  a = atoi(token);

  //set color
  int i = x;
  while(i<=y){
    strip.setPixelColor(i,r,g,b);
  }

  //set delay and draw
  if(r!=0 || g!=0 || b!=0)
    deley = DELEY;
  strip.show();
  
}

//String str;
void setup(){
  strip.begin();
  strip.show();
  Serial.begin(9600);
  
  set_color();
}

void loop(){
  char* cmd_in = read_cmd();
  decode_and_set(cmd_in);
}

