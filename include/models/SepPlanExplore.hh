#ifndef _SEPPLANEXPLORE_HH_
#define _SEPPLANEXPLORE_HH_

#include "models/C45Tree.hh"
#include "models/M5Tree.hh"
#include "models/LinearSplitsTree.hh"
#include "models/MultipleClassifiers.hh"

#include "models/Stump.hh"

#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <vector>
#include <set>
#include <map>

/** C4.5 decision stump class */
class SepPlanExplore: public Classifier {

public:

  // mode - re-build stump every step?  
  // re-build only on misclassifications? or rebuild every 'trainFreq' steps
  SepPlanExplore(int id, int modelType, int predType, int nModels, 
                 int trainMode, int trainFreq,
                 float featPct, float expPct, float treeThreshold, bool stoch,
                 float featRange, Random rng);

  SepPlanExplore(const SepPlanExplore&);
  virtual SepPlanExplore* getCopy();

  ~SepPlanExplore();

  virtual bool trainInstances(std::vector<classPair> &instances);
  virtual bool trainInstance(classPair &instance);
  virtual void testInstance(const std::vector<float> &input, std::map<float, float>* retval);
  virtual float getConf(const std::vector<float> &s);
  
  void initModels();

  bool SPEDEBUG;

private:

  const int id;
  const int modelType;
  const int predType;
  const int nModels;
  const int mode;
  const int freq;
  const float featPct;
  const float expPct;
  const float treeThresh;
  const bool stoch;
  const float featRange; 

  Random rng;

  Classifier* expModel;
  Classifier* planModel;

};


#endif
  
