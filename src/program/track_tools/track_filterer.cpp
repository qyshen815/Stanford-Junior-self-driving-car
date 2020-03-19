#include "track_manager.h"

#define MIN_TRACK_LENGTH (getenv("MIN_TRACK_LENGTH") ? atoi(getenv("MIN_TRACK_LENGTH")) : 0)

using namespace std;
using namespace track_manager;
using boost::shared_ptr;


int main(int argc, char** argv) {
  if(argc != 3) {
    cout << "Usage: " << argv[0] << " INPUT OUTPUT" << endl;
    return 0;
  }

  cout << "Min track length: " << MIN_TRACK_LENGTH << endl;
  
  cout << "Loading tracks from " << argv[1] << ", saving filtered tracks to " << argv[2] << endl;
  TrackManager tm(argv[1]);

  vector< shared_ptr<Track> > keep;
  keep.reserve(tm.tracks_.size());
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    if(MIN_TRACK_LENGTH > 0 && tm.tracks_[i]->frames_.size() < (size_t)MIN_TRACK_LENGTH)
      continue;

    keep.push_back(tm.tracks_[i]);
  }

  TrackManager out(keep);
  out.save(argv[2]);
  return 0;
}


  // template <typename T>
// class Options {
// public:
//   map<std::string, T> vals_;
  
//   void parse(int* argc, std::vector<std::string>* argv);
//   T getOption(const std::string& name) {return vals_[name];}
// };

// class ArgHandler {
// public:

//   std::map<std::string, std::string> string_options_;
//   std::map<std::string, bool> bool_options_;
//   std::map<std::string, double> double_options_;
  
//   void addOption(const std::string& type, const std::string& name, char abbrev = 0);
//   void parse(int argc, char** argv);
//   void parse(int* argc, std::vector<std::string>* argv);
  
//   std::string getStringOption(const std::string& name);
//   bool getBoolOption(const std::string& name);
//   double getDoubleOption(const std::string& name);
// };

// void ArgHandler::addOption(const std::string& type, const std::string& name, char abbrev = 0) {
//   if(type.compare("string") == 0)
//     string_options_.push_back(name);
//   else if(type.compare("bool") == 0)
//     bool_options_.push_back(name);
//   else if(type.compare("double") == 0)
//     double_options_.push_back(name);
//   else
//     assert(0);
// }


