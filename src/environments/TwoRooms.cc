/** \file TwoRooms.cc
    Implements a two room gridworld domain, with possible action delays or 
    multiple goals (with partial observability). 
    \author Todd Hester
*/

#include <environments/TwoRooms.hh>


Position::Position( unsigned x, unsigned y ): x(x), y(y) {}


inline bool operator == (const Position& position_01, const Position& position_02){
    return ((position_01.x == position_02.x) && (position_01.y == position_02.y));
}


TwoRooms::TwoRooms(Random &rand, bool stochastic, bool rewardType,
                   int actDelay, bool multiGoal):
  grid(defaultMap()),
  goal(1,1),
  goal2(9,1),
  negReward(rewardType),
  noisy(stochastic),
  actDelay(actDelay),
  multiGoal(multiGoal),
  rng(rand),
  doorway(2,5),
  sensation(2)
{
  reset();
}


TwoRooms::~TwoRooms() { delete grid; }

const std::vector<float> &TwoRooms::getSensation() const {
  //cout << "At state " << sensation[0] << ", " << sensation[1] << endl;

  return sensation;
}

float TwoRooms::apply(int action) {
//  cout << "Taking action " << static_cast<RoomAction>(action) << endl;

  int actUsed = action;

  if (actDelay > 0){
    actUsed = actHistory.front();
    actHistory.push_back(action);
    actHistory.pop_front();
  }

  if (actDelay == 0){

    // TODO
    // The action should have a equal chance to change to other state every N steps.
    // const RoomAction effect = noisy
    //   ? add_noise(static_cast<RoomAction>(actUsed))
    //   : static_cast<RoomAction>(actUsed);

    const RoomAction effect = static_cast<RoomAction>(actUsed);

    // Move or not the agent
    if ( !grid->wall(position.y, position.x, effect) )
    {
        cout << "\nInitial position = " << position.x << "," << position.y << endl;
        switch(effect) {
        case NORTH:
            cout << "Go north" << endl;
            ++position.y;
            break;
        case SOUTH:
            cout << "Go south" << endl;
            --position.y;
            break;
        case EAST:
            cout << "Go east" << endl;
            ++position.x;
            break;
        case WEST:
            cout << "Go west" << endl;
            --position.x;
            break;
        default:
            std::cerr << "Undefined action in TwoRooms::apply!!!\n";
            exit(EXIT_FAILURE);
        }
    cout << "Next position = " << position.x << "," << position.y << endl;
    }else {
        cout << "\nDont move. Tried to go ";
        switch( effect ){
        case NORTH:
            cout << "north.\n";
            break;
        case SOUTH:
            cout << "south.\n";
            break;
        case EAST:
            cout << "east.\n";
            break;
        case WEST:
            cout << "west.\n";
            break;
        default:
            std::cerr << "Undefined action in TwoRooms::apply!!!\n";
            exit(EXIT_FAILURE);
        }
    }
    return reward();
  }


  // Update sensation variable
  sensation[0] = static_cast<float>(position.y);
  sensation[1] = static_cast<float>(position.x);

  return 0;
}

// return the reward for this move
float TwoRooms::reward() {


//  if ( position == goal2 )
//    cout << "At goal 2, " << useGoal2 << endl;
//  if ( position == goal )
//    cout << "At goal 1, " << !useGoal2 << endl;


  if (negReward){
    // normally -1 and 0 on goal
    if (terminal())
      return 0;
    else 
      return -1;
    
  }else{

    // or we could do 0 and 1 on goal
    if (terminal())
      return 1;
    else 
      return 0;
  }
}



bool TwoRooms::terminal() const {
  // current position equal to goal??
  if (useGoal2)
    return position == goal2;
  else
    return position == goal;
}


void TwoRooms::reset() {
  // start randomly in right room
  position.y = rng.uniformDiscrete(0, grid->height() - 1 );
  position.x = rng.uniformDiscrete(6, grid->width() - 1);

  // a history of no_acts
  actHistory.clear();
  actHistory.assign(actDelay, -1);

  if (multiGoal){
    useGoal2 = rng.bernoulli(0.5);
    //cout << "goal2? " << useGoal2 << endl;
  }
  else {
    useGoal2 = false;
  }

  //ns = 4;
  //ew = 9;
}



int TwoRooms::getNumActions(){
  return 4;
}


const GridWorld *TwoRooms::defaultMap() {
  int width = 11;
  int height = 5;
  std::vector<std::vector<bool> > nsv(width, std::vector<bool>(height-1,false));
  std::vector<std::vector<bool> > ewv(height, std::vector<bool>(width-1,false));

  // put the wall between the two rooms
  for (int j = 0; j < height; j++){
    // skip doorway
    if (j == 2)
      continue;
    ewv[j][4] = true;
    ewv[j][5] = true;
  }

  nsv[5][1] = true;
  nsv[5][2] = true;

  // add a doorway
  doorway = Position(2, 5);

  // Print grid
  cout << GridWorld(height, width, nsv, ewv);
  return new GridWorld(height, width, nsv, ewv);
}

TwoRooms::RoomAction TwoRooms::add_noise(RoomAction action) {
  switch(action) {
  case NORTH:
  case SOUTH:
    return rng.bernoulli(0.8) ? action : (rng.bernoulli(0.5) ? EAST : WEST);
  case EAST:
  case WEST:
    return rng.bernoulli(0.8) ? action : (rng.bernoulli(0.5) ? NORTH : SOUTH);
  default:
    return action;
  }
}


void TwoRooms::randomize_goal() {
  const unsigned n = grid->height() * grid->width();
  unsigned index = rng.uniformDiscrete(1,n) - 1;
  goal = Position(index / grid->width(), index % grid->width());
}


void TwoRooms::getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat){
  
  minFeat->resize(sensation.size(), 0.0);
  maxFeat->resize(sensation.size(), 10.0);

  (*maxFeat)[0] = 5.0;

}

void TwoRooms::getMinMaxReward(float *minR,
                              float *maxR){
  if (negReward){
    *minR = -1.0;
    *maxR = 0.0;    
  }else{
    *minR = 0.0;
    *maxR = 1.0;
  }
}


std::vector<experience> TwoRooms::getSeedings() {

  // return seedings
  std::vector<experience> seeds;

  //if (true)
  // return seeds;
  // REMOVE THIS TO USE SEEDINGS

  // single seed of terminal state
  useGoal2 = false;
  actHistory.clear();
  actHistory.assign(actDelay, SOUTH);
  seeds.push_back(getExp(2,1,SOUTH));
  
  // possible seed of 2nd goal
  if (multiGoal){
    useGoal2 = true;
    actHistory.clear();
    actHistory.assign(actDelay, NORTH);
    seeds.push_back(getExp(3,1,NORTH));
  }

  // single seed of doorway
  actHistory.clear();
  actHistory.assign(actDelay, WEST);
  seeds.push_back(getExp(2,6,WEST));

  reset();

  return seeds;

}

experience TwoRooms::getExp(float s0, float s1, int a){

  experience e;

  e.s.resize(2, 0.0);
  e.next.resize(2, 0.0);

  position.x = s0;
  position.y = s1;

  e.act = a;
  e.s = getSensation();
  e.reward = apply(e.act);

  e.terminal = terminal();
  e.next = getSensation();

  /*
  cout << "Seed from " << e.s[0] << "," << e.s[1] << " a: " << e.act
       << " r: " << e.reward << " term: " << e.terminal << endl;
  */

  reset();

  return e;
}
