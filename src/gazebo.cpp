#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <common/Random.h>
#include <common/core.hh>
#include <agents/QLearner.hh>
#include <agents/Sarsa.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

using namespace std;


class RLVechicle{
    public:
    RLVechicle( ros::NodeHandle );
    ~RLVechicle();

    void simulate();
    void rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg);
    void sensationCallback(const std_msgs::Float32::ConstPtr& sensationMsg);

    private:
    int runSteps( Agent * agent );
    void resetEnvironment( );
    vector<experience> getSeedings( );
    experience getExperience( const unsigned action );

    ros::Rate loopRate;
    ros::Publisher actionPublisher;
    ros::Subscriber rewardSubscriber;
    ros::Subscriber sensationSubscriber;
    vector<float> sensation;
    float reward;

    enum VechicleAction{
        Backward = 0,
        Stop = 1,
        Forward = 2,
        NumActions = 3
    };
};



int main(int argc, char **argv){

    ros::init(argc, argv, "RLAgent");
    ros::NodeHandle node;

    RLVechicle roverSim( node );
    roverSim.simulate();

    return 0;
}


RLVechicle::RLVechicle( ros::NodeHandle node ): reward(-1), loopRate(2){
    sensation.resize(1,0);

    const unsigned bufferSize = 1;
    actionPublisher = node.advertise<std_msgs::Int32>("/rl/action", bufferSize);
    rewardSubscriber = node.subscribe("/rl/reward", bufferSize, &RLVechicle::rewardCallback, this);
    sensationSubscriber = node.subscribe("rl/sensation", bufferSize, &RLVechicle::sensationCallback, this);
}


RLVechicle::~RLVechicle(){
    ros::shutdown();
}


void RLVechicle::simulate()
{
    const int numactions = NumActions;
    const unsigned NUMTRIALS = 1;
    float alpha = 0.3;
    float discountfactor = 0.99;
    float initialvalue = 0.0;
    float epsilon = 0.1;
    float lambda = 0.1;
    int seed = 1;
    Random rng(1 + seed);

    float sumOfRewards = 0;
    for (unsigned j = 0; j < NUMTRIALS; ++j) {
        Agent* agent = new QLearner(numactions, discountfactor, initialvalue, alpha, epsilon, rng);
//        Agent* agent = new Sarsa(numactions, discountfactor, initialvalue, alpha, epsilon, lambda, rng);
//        agent->seedExp( getSeedings() );
        sumOfRewards += runSteps( agent );
        delete agent;
    }
    cout << "Avg(by trials) reward sum: " << (sumOfRewards / (float)NUMTRIALS) << endl;
}


vector<experience> RLVechicle::getSeedings( )
{
    vector<experience> expContainer;
    expContainer.push_back( getExperience(Backward) );
    expContainer.push_back( getExperience(Forward) );

    return expContainer;
}


experience RLVechicle::getExperience( const unsigned action )
{
    experience exp;
    exp.act = action;
    exp.s = sensation;
    std_msgs::Int32 vehicleAction;
    vehicleAction.data = exp.act;
    actionPublisher.publish( vehicleAction );
    exp.reward = reward;
    exp.terminal = false;
    loopRate.sleep();
    ros::spinOnce();
    exp.next = sensation;

    vehicleAction.data = Stop;
    actionPublisher.publish( vehicleAction );

    return exp;
}


int RLVechicle::runSteps( Agent * agent )
{
    unsigned MAXSTEPS = 2000;
    unsigned NUMEPISODES = 1;
    float sum = 0;

    for (unsigned i = 0; i < NUMEPISODES; ++i) {
        int step = 0;
//    rl_texplore::RLAction vehicleAction;
        std_msgs::Int32 vehicleAction;
        vehicleAction.data = agent->first_action( sensation );
        actionPublisher.publish( vehicleAction );
        sum += reward;

        while ( step < MAXSTEPS ) {
            // Sleep to work at loopRate frequency
            cout << "Step = " << step << endl;
            loopRate.sleep();
            ros::spinOnce();
            vehicleAction.data = agent->next_action( reward, sensation );
            actionPublisher.publish( vehicleAction );
            sum += reward;
            ++step;
            if(reward > -0.02){
                cout << "Terminal state (setpoint)." << endl;
                break;
            }
        }

        cout << "Episode: " << i+1 << endl;
        cout << "Sum: " << sum << endl;
    }
    resetEnvironment( );
    return sum;
}


void RLVechicle::resetEnvironment( )
{
    std_msgs::Int32 vehicleAction;
    vehicleAction.data = Stop;
    actionPublisher.publish( vehicleAction );
}

void RLVechicle::rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg)
{
    reward = rewardMsg->data;
}

void RLVechicle::sensationCallback(const std_msgs::Float32::ConstPtr& sensationMsg)
{
    sensation[0] = sensationMsg->data;
}
