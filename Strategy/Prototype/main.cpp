#include "strategy.h"
#include "ros/ros.h"
#include <time.h>


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "strategy");
	strategy STRATEGY;
	while(ros::ok())
	{
		STRATEGY.ComputeDistance();
		STRATEGY.FindBotsInsideCircle();
		STRATEGY.FirstOperation();

		while(STRATEGY.IsOutsideWhite())
		{
			time_t timer1, timer2;
			timer1 = time(NULL);
			STRATEGY.t_plan();
			STRATEGY.FirstOperation();
			timer2 = time(NULL) - timer1;
			while(timer2<=20)
				timer2 = time(NULL) - timer1;

		}
	}
}
