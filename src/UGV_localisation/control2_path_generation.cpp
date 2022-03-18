# include "ros/ros.h"
# include "prius_msgs/Control.h"
# include <iostream>

using namespace std;

int main (int argc, char **argv)
{
 ros::init(argc,argv,"control2_path_generation");
 
 ros::NodeHandle n;
 
 ros::Publisher instance = n.advertise<prius_msgs::Control>("/prius",1000);
 
 prius_msgs::Control msg;

 char input;

 msg.throttle = 0.0;
 msg.brake = 0.0; 
 msg.steer = 0.0;
 msg.shift_gears =1;
 
 while(ros::ok())
 {
  input =getchar();

  if (input == 'w')
  {
    msg.throttle = 1.0;  
    msg.brake = 0.0; 
    msg.steer = 0.0; 
  }
  else 
      {
         if (input == 's')
         {
           msg.brake = 1.0;
           msg.throttle = 0.0;
           msg.steer = 0.0;
         } 
         else 
            {
               if (input == 'a')
               {
                  msg.throttle = 1.0;
                  msg.brake = 0.0; 
                  msg.steer = 1.0;
               }
               else
                   {
                      if (input == 'd')
                      {
                       msg.throttle = 1.0;
                       msg.brake = 0.0; 
                       msg.steer = -1.0;
                      }
                      else
                          {
                            if (input == 'i')
                            {
                             msg.throttle = 0.0;
                             msg.brake = 0.0; 
                             msg.steer = 0.0;
                             msg.shift_gears =2;
                            }
                            else
                               {
                                 if (input == 'o')
                                 {
                                  msg.throttle = 0.0;
                                  msg.brake = 0.0; 
                                  msg.steer = 0.0;
                                  msg.shift_gears =1;
                                 }
                                 else 
                                     { 
                                       if (input =='p')
                                       {
                                        msg.throttle = 0.0;
                                        msg.brake = 0.0; 
                                        msg.steer = 0.0;
                                        msg.shift_gears =3;
                                       }  
                                     }
                               } 
                           }        
                    }              
               }  
        }
   instance.publish(msg);
}
 
 ros::spinOnce();
 
 return 0;
}
