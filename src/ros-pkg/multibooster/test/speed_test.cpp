#include <multibooster/multibooster.h>
#include <multibooster/synth_data.h>


int main(int argc, char** argv) { 
  SynthDatasetGenerator sdg;
  sdg.num_classes_ = 2;
  sdg.num_feature_spaces_ = 3;
  sdg.num_objs_ = 1000;
  sdg.num_bg_ = 5000;
  //sdg.feature_probability_ = 1;
  MultiBoosterDataset* mbd = sdg.generateDataset();
  MultiBooster mb(mbd);
  mb.verbose_ = true;
  mb.train(1, 0, 1000);
  //  WCTree wct(mb.pwcs_, vector<WeakClassifier*>(), 16);
}
