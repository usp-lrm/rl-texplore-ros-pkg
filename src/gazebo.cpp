#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <common/Random.h>
#include <common/core.hh>
#include <agents/QLearner.hh>
#include <agents/Sarsa.hh>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>

using namespace std;


class RLVehicle{
    public:
    RLVehicle( ros::NodeHandle );
    ~RLVehicle();

    void simulate();
    void rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg);
    void positionCallback(const geometry_msgs::Point32::ConstPtr& positionMsg);
    void velocityCallback(const std_msgs::Int32::ConstPtr& velocityMsg);
    void steeringCallback(const std_msgs::Int32::ConstPtr& steeringMsg);

    private:
    int runSteps( Agent * agent );
    void resetEnvironment( );
    vector<experience> getSeedings( );
    experience getExperience( const unsigned action );

    ros::Rate loopRate;
    ros::Publisher actionPublisher;
    ros::Subscriber rewardSubscriber;
    ros::Subscriber positionSubscriber, velocitySubscriber, steeringSubscriber;
    vector<float> stateObservation;
    vector<float> oldObservation;
    float reward;

    enum VechicleAction{
        NumActions = 5
    };
};



int main(int argc, char **argv){

    ros::init(argc, argv, "RLAgent");
    ros::NodeHandle node;

    RLVehicle roverSim( node );
    roverSim.simulate();

    return 0;
}


RLVehicle::RLVehicle( ros::NodeHandle node ): reward(-1), loopRate(2){
    stateObservation.resize(5,0);
    oldObservation.resize(5,0);

    const unsigned bufferSize = 1;
    actionPublisher = node.advertise<std_msgs::Int32>("/rl/action", bufferSize);
    rewardSubscriber = node.subscribe("/rl/reward", bufferSize, &RLVehicle::rewardCallback, this);
    positionSubscriber = node.subscribe("/rl/state/position", bufferSize, &RLVehicle::positionCallback, this);
    velocitySubscriber = node.subscribe("/rl/state/velocity", bufferSize, &RLVehicle::velocityCallback, this);
    steeringSubscriber = node.subscribe("/rl/state/steering", bufferSize, &RLVehicle::steeringCallback, this);
}


RLVehicle::~RLVehicle(){
    ros::shutdown();
}


void RLVehicle::simulate()
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


vector<experience> RLVehicle::getSeedings( )
{
    vector<experience> expContainer;
    expContainer.push_back( getExperience(0) );
    expContainer.push_back( getExperience(1) );

    return expContainer;
}


experience RLVehicle::getExperience( const unsigned action )
{
    experience exp;
    exp.act = action;
    exp.s = stateObservation;
    std_msgs::Int32 vehicleAction;
    vehicleAction.data = exp.act;
    actionPublisher.publish( vehicleAction );
    exp.reward = reward;
    exp.terminal = false;
    loopRate.sleep();
    ros::spinOnce();
    exp.next = stateObservation;

    vehicleAction.data = 9;
    actionPublisher.publish( vehicleAction );

    return exp;
}


int RLVehicle::runSteps( Agent * agent )
{
    unsigned MAXSTEPS = 1000;
    unsigned NUMEPISODES = 1;
    float sum = 0;

    for (unsigned i = 0; i < NUMEPISODES; ++i) {
        int step = 0;
//    rl_texplore::RLAction vehicleAction;
        std_msgs::Int32 vehicleAction;
        vehicleAction.data = agent->first_action( oldObservation );
        actionPublisher.publish( vehicleAction );
        oldObservation = stateObservation;
        sum += reward;

        while ( step < MAXSTEPS ) {
            // Sleep to work at loopRate frequency
            cout << "Step = " << step << endl;
            loopRate.sleep();
            ros::spinOnce();
            vehicleAction.data = agent->next_action( reward, oldObservation );
            actionPublisher.publish( vehicleAction );
            oldObservation = stateObservation;
            sum += reward;
            ++step;
            if(reward >= - 0.1){
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


void RLVehicle::resetEnvironment( )
{
    std_msgs::Int32 vehicleAction;
    vehicleAction.data = 0;
    actionPublisher.publish( vehicleAction );
}

void RLVehicle::rewardCallback(const std_msgs::Float32::ConstPtr& rewardMsg)
{
    reward = rewardMsg->data;
}

void RLVehicle::positionCallback(const geometry_msgs::Point32::ConstPtr& positionMsg)
{
    stateObservation[0] = static_cast<int>( positionMsg->x );
    stateObservation[1] = static_cast<int>( positionMsg->y );
    stateObservation[2] = static_cast<int>( positionMsg->z );
}

void RLVehicle::velocityCallback(const std_msgs::Int32::ConstPtr& velocityMsg)
{
    stateObservation[3] = velocityMsg->data;
}

void RLVehicle::steeringCallback(const std_msgs::Int32::ConstPtr& steeringMsg)
{
    stateObservation[4] = steeringMsg->data;
}
