#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <common/Random.h>
#include <common/core.hh>
#include <agents/QLearner.hh>

#include <ros/ros.h>
#include <rl_texplore/RLAction.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

using namespace std;


struct RLVariables{
    static float reward;
    static float sensation;
};
float RLVariables::reward = -1;
float RLVariables::sensation = 0;

void rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg);
void sensationCallback(const std_msgs::Float32::ConstPtr& sensationMsg);

int runAlgorithm( Agent * agent, ros::Publisher actionPublisher );
void resetEnvironment( ros::Publisher actionPublisher );


int main(int argc, char **argv){

    ros::init(argc, argv, "RLAgent");
    ros::NodeHandle node;


    const unsigned bufferSize = 1;
    ros::Publisher actionPublisher = node.advertise<std_msgs::Int32>("/rl/action", bufferSize);
    ros::Subscriber rewardSubscriber = node.subscribe("/rl/reward", bufferSize, rewardCallback);
    ros::Subscriber sensationSubscriber = node.subscribe("rl/sensation", bufferSize, sensationCallback);

    const int numactions = 3;
    const unsigned NUMTRIALS = 1;
    float alpha = 0.3;
    float discountfactor = 0.99;
    float initialvalue = 0.0;
    float epsilon = 0.1;
    int seed = 1;
    Random rng(1 + seed);

    cout << "alpha: " << alpha << endl;
    cout << "discountfactor: " << discountfactor << endl;
    cout << "initialvalue: " << initialvalue << endl;
    cout << "epsilon: " << epsilon << endl;


    float sumOfRewards = 0;
    cout << "Agent: QLearner" << endl;
    for (unsigned j = 0; j < NUMTRIALS; ++j) {
        Agent* agent = new QLearner(numactions, discountfactor, initialvalue, alpha, epsilon, rng);
//        std::vector<experience> seeds;
//        agent->seedExp( seeds );
        sumOfRewards += runAlgorithm( agent, actionPublisher );
        delete agent;
    }
    cout << "Avg(by trials) reward sum: " << (sumOfRewards / (float)NUMTRIALS) << endl;

    ros::shutdown();
    return 0;
}




int runAlgorithm( Agent * agent,  ros::Publisher actionPublisher )
{
    unsigned MAXSTEPS = 1000;
    unsigned NUMEPISODES = 1;
    float sum = 0;

    // ROS loop rate in Hz
    ros::Rate loop_rate(10);

    for (unsigned i = 0; i < NUMEPISODES; ++i) {
        int steps = 0;
//    rl_texplore::RLAction vehicleAction;
        std_msgs::Int32 vehicleAction;
        vehicleAction.data = agent->first_action( std::vector<float>(RLVariables::sensation) );
        actionPublisher.publish( vehicleAction );
        sum += RLVariables::reward;

        while ( steps < MAXSTEPS ) {
            // Sleep to work at loop_rate frequency
            loop_rate.sleep();
            ros::spinOnce();
            vehicleAction.data = agent->next_action( RLVariables::reward, std::vector<float>(RLVariables::sensation) );
            actionPublisher.publish( vehicleAction );
            sum += RLVariables::reward;
            ++steps;
            if(RLVariables::reward > -0.01){
                agent->last_action(RLVariables::reward);
                resetEnvironment( actionPublisher );
                break;
            }
        }
        agent->last_action(RLVariables::reward);
        resetEnvironment( actionPublisher );

        cout <<"Episode: " << i+1 << endl;
        cout <<"Sum: " << sum << endl;
    }
  return sum;
}


void resetEnvironment( ros::Publisher actionPublisher )
{
    std_msgs::Int32 vehicleAction;
    vehicleAction.data = 0;
    actionPublisher.publish( vehicleAction );
}

void rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg)
{
    RLVariables::reward = rewardMsg->data;
}

void sensationCallback(const std_msgs::Float32::ConstPtr& sensationMsg)
{
    RLVariables::sensation = sensationMsg->data;
}
