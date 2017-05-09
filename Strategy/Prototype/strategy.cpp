#include <time.h>                                     //confirm x and y axis from Aditi
#include <math.h>
#include <utility>
#include <set>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "strategy.h"
#include "ros/ros.h"

#define PI 3.14159
#define MIN_DIST 9

using namespace std;

int strategy::IsOutsideWhite()
{
	int centerBotID;
	p temp = *ClosestBot.begin();
	centerBotID = temp.second;
	char topic_name[40];
	sprintf(topic_name, "robot%d/odom", centerBotID);
	ros::Rate loop_rate(10);
        sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);
	ros::spinOnce();
	loop_rate.sleep();
	if(centerY>=MIN_DIST)
		return 1;
	else
		return 0;
}

void strategy::posecallback(nav_msgs::Odometry::ConstPtr msg){

	posX = msg->pose.pose.position.y;
	posY = -msg->pose.pose.position.x;
	x = msg->pose.pose.orientation.x;
	y = msg->pose.pose.orientation.y;
	z = msg->pose.pose.orientation.z;
	w = msg->pose.pose.orientation.w;
}

void strategy::centercallback(nav_msgs::Odometry::ConstPtr msg){

	centerX = msg->pose.pose.position.y;
	centerY = -msg->pose.pose.position.x;
	x = msg->pose.pose.orientation.x;
	y = msg->pose.pose.orientation.y;
	z = msg->pose.pose.orientation.z;
	w = msg->pose.pose.orientation.w;
}


void strategy::find_herd_bots()
{
	float max;
	for(int i=4;i<14;i++){

		char topic_name[40];
		sprintf(topic_name, "robot%d/odom", i);
		sub = n.subscribe(topic_name, 1, &strategy::posecallback,this);
		ros::spinOnce();
		double y = posY;
		distance_bots.push_back(y);
	}

	max=-FLT_MAX;
	no1 = no2 = 0;
	for(int i=4;i<14;i++){
		if(distance_bots[i]<0){
			if(distance_bots[i]>=max){
				no2 = no1;
				max=distance_bots[i];
				no1 = i;
			}
		}
	}
	herd_bots(no1, no2);
}

void strategy::herd_bots(int no1, int no2)                              //go to the no1 and no2 bot and tap for turning them towards the center
{
	int bots[2] = {no1, no2};
	int z=0;
	while(z<=1)
	{
		double yaw=0, pitch=0, roll=0;
		char topic_name[40];
		sprintf(topic_name, "robot%d/odom", bots[z]);
		sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
	  ros::spinOnce();
		GetEulerAngles(&yaw, &pitch, &roll);

	 	pub = n.advertise<nav_msgs::Odometry>(topic_name, 1000);
		nav_msgs::Odometry msg;
		double theta1 = atan((10 - posY)/(10 - posX));
		double theta2 = atan((10 - posY)/(-10 - posX));

		/*if(yaw>=theta2 && yaw<=theta1)
			//do nothing*/
		if((yaw>=angle(theta1+PI) && yaw<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (yaw>=angle(theta1+PI) || yaw<=angle(theta2+PI)) ) )
		{
			//180 degree turn, come in front
			yaw = angle(yaw + PI);
			toQuaternion(yaw,pitch,roll);
	    msg.pose.pose.orientation.x=x;
	    msg.pose.pose.orientation.y=y;
	    msg.pose.pose.orientation.z=z;
	    msg.pose.pose.orientation.w=w;
	    pub.publish(msg);
		}
		else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
		{
			//45 degree turn, tap from top
			yaw = angle(yaw-PI/4);
			toQuaternion(yaw,pitch,roll);
	    msg.pose.pose.orientation.x=x;
	    msg.pose.pose.orientation.y=y;
	    msg.pose.pose.orientation.z=z;
	    msg.pose.pose.orientation.w=w;
	    pub.publish(msg);
		}
		else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
		{
			//90 degree turn, tap twice
			yaw = angle(yaw-PI/2);
			toQuaternion(yaw,pitch,roll);
	    msg.pose.pose.orientation.x=x;
	    msg.pose.pose.orientation.y=y;
	    msg.pose.pose.orientation.z=z;
	    msg.pose.pose.orientation.w=w;
	    pub.publish(msg);

		}
		else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
		{
			//first rotate by 180 degrees and then turn by 45 degrees
			yaw = angle(yaw-PI-PI/4);
			toQuaternion(yaw,pitch,roll);
	    msg.pose.pose.orientation.x=x;
	    msg.pose.pose.orientation.y=y;
	    msg.pose.pose.orientation.z=z;
	    msg.pose.pose.orientation.w=w;
	    pub.publish(msg);
		}
		z++;
	}

}


void strategy::ComputeDistance(){                                     //computes distance of every bot from the green line

	for(int i=4;i<14;i++){

		char topic_name[40];
		sprintf(topic_name, "robot%d/odom", i);
		sub = n.subscribe(topic_name, 1, &strategy::posecallback,this);
		ros::spinOnce();
		double y = 10 - posY;
		int BotID = i;
		ClosestBot.insert(make_pair(y,BotID));

	}
}


void strategy::FindBotsInsideCircle(){                               //to find the bot inside the 5m circle

	int centerBotID;

	p temp = *ClosestBot.begin();
	centerBotID = temp.second;

	int count=4;

	while(count<14){

		if(count!=centerBotID){

			char topic_name[40];
			sprintf(topic_name, "robot%d/odom", centerBotID);
			sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);
			ros::spinOnce();
			char topic_name2[40];
			sprintf(topic_name2, "robot%d/odom", count);
			sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);
			ros::spinOnce();
		}


		if((pow(posX-centerX,2) + pow(posY-centerY,2) - 25) <= 0 && count!=centerBotID){

			BotsInsideCircle.push_back(count);
		}
		count++;
	}
}

void strategy::FirstOperation(){                                          //to decide the first operation of the center bot

	double yaw=0, pitch=0, roll=0;
	int centerBotID;

	p temp = *ClosestBot.begin();
	centerBotID = temp.second;

	char topic_name[40];
	sprintf(topic_name, "robot%d/odom", centerBotID);
	sub = n.subscribe(topic_name, 1000, &strategy::centercallback,this);
        ros::spinOnce();
	GetEulerAngles(&yaw, &pitch, &roll);

        pub = n.advertise<nav_msgs::Odometry>(topic_name, 1000);
	nav_msgs::Odometry msg;
	double theta1 = atan((10 - centerY)/(10 - centerX));
	double theta2 = atan((10 - centerY)/(-10 - centerX));

	/*if(yaw>=theta2 && yaw<=theta1)
		//do nothing*/
	if((yaw>=angle(theta1+PI) && yaw<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (yaw>=angle(theta1+PI) || yaw<=angle(theta2+PI)) ) )
	{
		//180 degree turn, come in front
		yaw = angle(yaw + PI);
		toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
	}
	else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
	{
		//45 degree turn, tap from top
		yaw = angle(yaw-PI/4);
		toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
	}
	else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
	{
		//90 degree turn, tap twice
		yaw = angle(yaw-PI/2);
		toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);

	}
	else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
	{
		//first rotate by 180 degrees and then turn by 45 degrees
		yaw = angle(yaw-PI-PI/4);
		toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
	}
}


void strategy::GetEulerAngles( double* yaw, double* pitch, double* roll)
  {
      const double w2 = w*w;
      const double x2 = x*x;
      const double y2 = y*y;
      const double z2 = z*z;
      const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
      const double abcd = w*x + y*z;
      const double eps = 1e-7;                        // TODO: pick from your math lib instead of hardcoding.
      const double pi = 3.14159265358979323846;       // TODO: pick from your math lib instead of hardcoding.
      if (abcd > (0.5-eps)*unitLength)
      {
          *yaw = 2 * atan2(y, w);
          *pitch = pi;
          *roll = 0;
      }
      else if (abcd < (-0.5+eps)*unitLength)
      {
          *yaw = -2 * ::atan2(y, w);
          *pitch = -pi;
          *roll = 0;
      }
      else
      {
          const double adbc = w*z - x*y;
          const double acbd = w*y - x*z;
          *yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
          *pitch = ::asin(2*abcd/unitLength);
          *roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
      }

  }


void strategy::toQuaternion(double pitch, double roll, double yaw)
{
	double t0 = cos(yaw * 0.5);
	double t1 = sin(yaw * 0.5);
	double t2 = cos(roll * 0.5);
	double t3 = sin(roll * 0.5);
	double t4 = cos(pitch * 0.5);
	double t5 = sin(pitch * 0.5);

	w = t0 * t2 * t4 + t1 * t3 * t5;
	x = t0 * t3 * t4 - t1 * t2 * t5;
	y = t0 * t2 * t5 + t1 * t3 * t4;
	z = t1 * t2 * t4 - t0 * t3 * t5;

}


  float strategy::dist_whitel(){

  	double yaw,pitch,roll;
  	GetEulerAngles(&yaw,&pitch,&roll);
  	float orient=yaw;


  if(orient>=0 && orient<PI/2){
      return 10-posY;
  }

  if(orient>=PI/2 && orient>PI){
      return 10+posY;
  }

  if(orient<0 && orient>-PI/2){
  	if(10-posX<10+posY)
      return 10-posX;
  	else
  		return 10+posY;
  }

  if(orient>-PI && orient<-PI/2){
  	if(10+posY<10+posX)
      return 10+posY;
  	else
  		return 10+posX;
  }

}

float strategy::angle(float ang){

  if(ang>=0 && ang<=PI)
    return ang;
  else if(ang>PI)
    return ang-2*PI;
  else if(ang<-PI)
    return ang+2*PI;

}


void strategy::action(int bot_no){
  float theta1,theta2,orient;
  char topic_name[40];
  sprintf(topic_name, "robot%d/odom", bot_no);

  theta1=angle(atan((centerY-posY)/(centerX-posX))-PI/4);
  theta2=angle(theta1 + PI/2);

  double yaw,pitch,roll;
  GetEulerAngles(&yaw,&pitch,&roll);
  orient=yaw;


  pub = n.advertise<nav_msgs::Odometry>(topic_name, 1);
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = posY;
  msg.pose.pose.position.y = -posX;

  if( (orient>theta2 && orient<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (orient>theta2 || orient<angle(theta2+PI/4) ) ) ){
    //one 45 degree rotation anticlockwise
  	yaw=angle(yaw+PI/4);
    toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
  }
  else if( (orient>=angle(theta1+PI) && orient<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (orient>=angle(theta1+PI) || orient<=angle(theta2+PI)) ) ){
    //turn 180 degree
    yaw=angle(yaw+PI);
    toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
  }
  else if( (orient<theta1 && orient>=angle(theta1-PI/4)) || ( theta1*angle(theta1-PI/4)<0 && (orient>angle(theta1-PI/4) || orient<theta1 ) ) ){
    //one 180 degree turn and then one 45 degree anti clockwise rotation
    yaw=angle(yaw+5*PI/4);
    toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
  }
  else if( (orient>angle(theta2+PI/4) && orient<=angle(theta2+PI/2)) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (orient>angle(theta1+PI/4) || orient<angle(theta2+PI/2) ) )){
    //two 45 degree anticlockwise rotations
    yaw=angle(yaw+PI/2);
    toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
  }
  else if( (orient<angle(theta1-PI/4) && orient>=angle(theta1-PI/2)) || ( angle(theta1-PI/2)*angle(theta1-PI/4)<0 && (orient>angle(theta1-PI/2) || orient<angle(theta1-PI/4) ) )){
    //one 180 degree turn and two 45 degree anticlockwise turn
    yaw=angle(yaw+3*PI/2);
    toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    pub.publish(msg);
  }

}


void strategy::t_plan(){
 //no for the target bot and the vector for the bots inside the circle
  int centerBotID;
  p temp = *ClosestBot.begin();
  centerBotID = temp.second;

  int size=BotsInsideCircle.size();

  char topic_name[40];
  sprintf(topic_name, "robot%d/odom", centerBotID);
  sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);
  ros::spinOnce();                                           // suscribe to the publisher to get the coordinates of the target bot

  float dis,min;
  int bot_no=BotsInsideCircle[0];
  char topic_name2[40];

  for(int i=0;i<size;i++){

    //suscribe and get the coordinates of the bot with the no a[i]
    //get the orintation of the bot from x axis

	sprintf(topic_name2, "robot%d/odom", BotsInsideCircle[i]);
	sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);
	ros::spinOnce();

    dis=dist_whitel();
    if(i==0)
      min=dis;
    else if(dis<min){
      min=dis;
      bot_no=BotsInsideCircle[i];
    }

  }
  //suscribe and get the position of the selected bot no
  sprintf(topic_name2, "robot%d/odom", bot_no);
  sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);
  ros::spinOnce();
  action(bot_no);


}
