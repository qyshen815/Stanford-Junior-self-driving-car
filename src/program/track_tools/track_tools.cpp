#include <multibooster_support.h>
#include <track_manager.h>
#include <track_manager_cached_descriptors.h>
#include <set>
#include <vector>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;

string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << " track_tools --status TM [TM ...]" << endl;
  oss << endl;
  oss << " track_tools --latex-table --training TM [TM ...] --testing TM [TM ...]" << endl;
  oss << endl;
  oss << " track_tools --filter_only LABEL [LABEL ...] SRC DEST" << endl;
  oss << "    Selects only those tracks with the given labels from SRC and saves a copy of them in DEST." << endl;
  oss << endl;
  oss << " track_tools --join DEST SRC1 SRC2 [SRC3 ...]" << endl;
  oss << endl;
  oss << " track_tools --evensplit SRC DEST1 DEST2 [DEST3 ...]" << endl;
  oss << "    Splits SRC evenly into DEST1, DEST2, etc." << endl;
  oss << endl;
  oss << " track_tools --split SRC PCT1 DEST1 PCT2 DEST2 [PCT3 DEST3 ...] " << endl;
  oss << "    Splits SRC into DEST1, DEST2, etc., according to PCT1, PCT2, etc." << endl;
  oss << "    PCT_i \\in [0, 1]; \\sum_i PCT_i = 1." << endl;
  oss << endl;
  oss << " track_tools --extract-seed-labels NUM SRC SEED REMAINDER" << endl;
  oss << "    Randomly selects NUM tracks of each label (except background & unlabeled) from SRC and saves to SEED." << endl;
  oss << "    The rest are saved in REMAINDER." << endl;
  oss << endl;
  oss << " track_tools --label-all LABEL SRC DEST" << endl;
  oss << "    Label all tracks in SRC as LABEL, then save in DEST.  SRC is not changed." << endl;
  oss << endl;
  oss << " track_tools --label-all-cached LABEL SRC DEST" << endl;
  oss << "    Label all cached track descriptors in SRC as LABEL, then save in DEST.  SRC is not changed." << endl;
  oss << endl;
  oss << " track_tools --cache-descriptors SRC DEST" << endl;
  oss << "    Computes descriptors for SRC and saves them in DEST.  DEST must be a .tm.mbd file." << endl;
  oss << endl;
  oss << " track_tools --join-tm-mbd [--no-unlabeled] CACHED [CACHED ...] MBD" << endl;
  oss << "    Many .tm.mbd files are joined into a single .mbd." << endl;
  oss << endl;
  oss << " track_tools --check-for-duplicates TM [TM ...]" << endl;
  oss << "    Checks for duplicate tracks in track managers." << endl;
  oss << endl;
  oss << " track_tools --label-diff TM TM" << endl;
  oss << "    Reports differences between otherwise identical TMs." << endl;
  oss << endl;

  return oss.str();
}

void labelDiff(const string& path1, const string& path2)
{
  cout << "Loading " << path1 << endl;
  TrackManager tm1(path1);
  tm1.sortTracks();
  cout << "Loading " << path2 << endl;
  TrackManager tm2(path2);
  tm2.sortTracks();

  if(tm1.tracks_.size() != tm2.tracks_.size()) {
    cout << "Track managers have different numbers of tracks." << endl;
    cout << "Aborting." << endl;
    return;
  }

  double num_diff = 0;
  for(size_t i = 0; i < tm1.tracks_.size(); ++i) {
    //cout << "Track " << i << endl;
    Track& tr1 = *tm1.tracks_[i];
    Track& tr2 = *tm2.tracks_[i];
    
    assert(tr1.frames_.size() == tr2.frames_.size());

    // -- Make sure they're the same, other than the label.
    string label = tr1.label_;
    tr1.label_ = tr2.label_;
    if(tr1 != tr2) {
      cout << "Track " << i << " does not appear to match in the two .tm files." << endl;
      cout << tr1.getMeanNumPoints() << " " << tr2.getMeanNumPoints() << endl;
      assert(tr1 == tr2);
    }
    tr1.label_ = label;
    
    if(tr1.label_.compare(tr2.label_) != 0) { 
      cout << "Track " << i+1 << " differs in labeling: " << tr1.label_ << " vs " << tr2.label_ << endl;
      ++num_diff;
    }
  }

//   double num_diff = 0;
//   for(size_t i = 0; i < tm1.tracks_.size(); ++i) {
//     Track& tr1 = *tm1.tracks_[i];
//     // -- Find the matching track.
//     string label = tr1.label_;
//     int idx = -1;
//     for(size_t j = 0; j < tm2.tracks_.size(); ++j) {
//       tr1.label_ = tm2.tracks_[j]->label_;
//       if(tr1 == *tm2.tracks_[j]) { 
// 	idx = j;
// 	break;
//       }
//     }
//     assert(idx != -1);
//     Track& tr2 = *tm2.tracks_[idx];
//     tr1.label_ = label;
//     assert(tr1.frames_.size() == tr2.frames_.size());

//     // -- Compare the labels.
//     if(tr1.label_.compare(tr2.label_) != 0) { 
//       cout << "Track " << i << " (" << idx << ") differs in labeling: " << tr1.label_ << " vs " << tr2.label_ << endl;
//       ++num_diff;
//     }
//   }

  cout << "Total tracks: " << tm1.tracks_.size() << endl;
  cout << "Number that are different: " << num_diff << endl;
  cout << "Percentage that are different: " << num_diff / (double)tm1.tracks_.size() << endl;
}

void findDuplicates(const vector<TrackManager*>& tms, size_t tm_index, size_t track_index,
		    vector<size_t> *duplicate_tm_indices, vector<size_t> *duplicate_track_indices)
{
  Track& track = *tms[tm_index]->tracks_[track_index];
  for(size_t i = 0; i < tms.size(); ++i) {
    for(size_t j = 0; j < tms[i]->tracks_.size(); ++j) {
      if(j == track_index && i == tm_index)
	continue;
      if(track == *tms[i]->tracks_[j]) { 
	duplicate_tm_indices->push_back(i);
	duplicate_track_indices->push_back(j);
      }
    }
  }
}

void checkForDuplicates(const vector<string>& paths)
{
  // -- Load data.
  vector<TrackManager*> tms;
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << "Loading " << paths[i] << endl;
    tms.push_back(new TrackManager(paths[i]));
  }

  // -- Check for duplicates.
  for(size_t i = 0; i < tms.size(); ++i) {
    cout << "Looking for duplicates in " << paths[i] << endl;
    for(size_t j = 0; j < tms[i]->tracks_.size(); ++j) {
      vector<size_t> tm_indices;
      vector<size_t> track_indices;
      findDuplicates(tms, i, j, &tm_indices, &track_indices);
      assert(tm_indices.size() == track_indices.size());
      if(!tm_indices.empty()) {
	cout << "Found duplicate!" << endl;
	cout << " " << paths[i] << ":" << j << " has " << tm_indices.size() << " dupe(s) at:" << endl;
	for(size_t k = 0; k < tm_indices.size(); ++k) {
	  cout << "   " << paths[tm_indices[k]] << ":" << track_indices[k] << endl;
	}
      }
    }
  }

  // -- Clean up.
  for(size_t i = 0; i < tms.size(); ++i) {
    delete tms[i];
  }
}

void joinCachedDescriptors(const vector<string>& input_paths, const string& output_path, bool allow_unlabeled)
{

  vector<Object*> objs;
  vector<string> descriptor_names;
  for(size_t i = 0; i < input_paths.size(); ++i) {
    TrackManagerCachedDescriptors tmcd(input_paths[i]);
    vector<Object*> tmcd_objs = tmcd.copyObjects();
    objs.reserve(objs.size() + tmcd_objs.size());
    
    // -- If we're not taking unlabeled objects, then delete them.
    for(size_t j = 0; j < tmcd_objs.size(); ++j) { 
      if(allow_unlabeled || tmcd_objs[j]->label_ != -2)
	objs.push_back(tmcd_objs[j]); // These are freed by MultiBoosterDataset.
      else
	delete tmcd_objs[j];
    }

    descriptor_names = tmcd.descriptor_map_.getIdToNameMapping(); // Constant for all tmcds.  Class names might not be.
  }
  
  MultiBoosterDataset mbd(getClassNames(), descriptor_names);
  mbd.setObjs(objs);
  cout << mbd.status() << endl;
  mbd.save(output_path);
}

void cacheDescriptors(const TrackManager& tm, const string& path)
{
  TrackManagerCachedDescriptors tmcd(tm, getClassNames());
  tmcd.save(path);
}

void filter(const vector<string>& labels, const string& source, const string& destination) {
  TrackManager src(source);
  TrackManager dest;
  for(size_t i = 0; i < src.tracks_.size(); ++i) {
    for(size_t j = 0; j < labels.size(); ++j) { 
      if(src.tracks_[i]->label_.compare(labels[j]) == 0) { 
	dest.insertTrack(src.tracks_[i]);
	break;
      }
    }
  }

  dest.save(destination);
  cout << "Saved filtered tracks in " << destination << endl;
}

void getCounts(const vector<string>& filenames, map<string, int>* tracks_ptr, map<string, int>* frames_ptr) {
  map<string, int>& tracks = *tracks_ptr;
  map<string, int>& frames = *frames_ptr;
  
  for(size_t i = 0; i < filenames.size(); ++i) {
    ifstream file;
    cout << "Counting up " << filenames[i] << endl;
    file.open(filenames[i].c_str());
    assert(file.is_open());
    string line;
    while(!file.eof()) {
      getline(file, line);
      if(line.compare("Track") == 0) {
	getline(file, line);
	getline(file, line);
	getline(file, line);
	getline(file, line);
	string label = line;
	++tracks[label];
	
	getline(file, line);
	getline(file, line);
	getline(file, line);
	getline(file, line);
	string num_frames = line;
	frames[label] += atoi(num_frames.c_str());
      }
    }
    file.close();
  }
}

void status(const vector<string>& filenames) {
  map<string, int> tracks;
  map<string, int> frames;
  getCounts(filenames, &tracks, &frames);
  
  // -- Display the counts.
  string labelstr = "Label";
  labelstr.resize(30, ' ');
  cout << labelstr << ": \tTracks\t\tFrames" << endl;
  cout << "--------------------------------------------------------------------------------" << endl;

  int total_tracks = 0;
  int total_frames = 0;
  map<string, int>::iterator it;
  for(it = tracks.begin(); it != tracks.end(); ++it) {
    string label = it->first;
    int num_tracks = it->second;
    int num_frames = frames[label];
    label.resize(30, ' ');
    cout << label << ": \t" << num_tracks << "\t\t" << num_frames << endl;
    total_tracks += num_tracks;
    total_frames += num_frames;
  }

  cout << "                              :" << endl;
  string total = "Total";
  total.resize(30, ' ');
  cout << total << ": \t" << total_tracks << "\t\t" << total_frames << endl;
  
}

string computeLatexTable(const vector<string>& training_sets, const vector<string>& testing_sets) {
  // -- Collect training set counts.
  map<string, int> training_tracks;
  map<string, int> training_frames;
  getCounts(training_sets, &training_tracks, &training_frames);

  map<string, int>::iterator it;
  int total_training_tracks = 0;
  int total_training_frames = 0;
  for(it = training_frames.begin(); it != training_frames.end(); ++it) {
    string label = it->first;
    if(label.compare("unlabeled") == 0)
      continue;

    total_training_frames += training_frames[label];
    total_training_tracks += training_tracks[label];
  }
  
  // -- Collect test set counts.
  map<string, int> testing_tracks;
  map<string, int> testing_frames;
  getCounts(testing_sets, &testing_tracks, &testing_frames);

  int total_testing_tracks = 0;
  int total_testing_frames = 0;
  for(it = testing_frames.begin(); it != testing_frames.end(); ++it) {
    string label = it->first;
    if(label.compare("unlabeled") == 0)
      continue;

    total_testing_frames += testing_frames[label];
    total_testing_tracks += testing_tracks[label];
  }
  
  // -- Write out table.  
  ostringstream oss;
  oss << "\\begin{tabular}{|l|r|r|r|r|r|}" << endl;
  oss << "\\multicolumn{6}{c}{\\textbf{Number of tracks}} \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Set & Car & Pedestrian & Bicyclist & Background & All \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Training"
      << " & " << training_tracks["car"]
      << " & " << training_tracks["pedestrian"]
      << " & " << training_tracks["bicyclist"]
      << " & " << training_tracks["background"]
      << " & " << total_training_tracks << " \\\\" << endl;
  oss << "Testing"
      << " & " << testing_tracks["car"]
      << " & " << testing_tracks["pedestrian"]
      << " & " << testing_tracks["bicyclist"]
      << " & " << testing_tracks["background"]
      << " & " << total_testing_tracks << " \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Total"
      << " & " << testing_tracks["car"] + training_tracks["car"]
      << " & " << testing_tracks["pedestrian"] + training_tracks["pedestrian"]
      << " & " << testing_tracks["bicyclist"] + training_tracks["bicyclist"]
      << " & " << testing_tracks["background"] + training_tracks["background"]
      << " & " << total_testing_tracks + total_training_tracks << " \\\\" << endl;
  oss << "\\hline" << endl;

  oss << "\\multicolumn{6}{c}{} \\\\" << endl;
  oss << "\\multicolumn{6}{c}{\\textbf{Number of frames}} \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Set & Car & Pedestrian & Bicyclist & Background & All \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Training"
      << " & " << training_frames["car"]
      << " & " << training_frames["pedestrian"]
      << " & " << training_frames["bicyclist"]
      << " & " << training_frames["background"]
      << " & " << total_training_frames << " \\\\" << endl;
  oss << "Testing"
      << " & " << testing_frames["car"]
      << " & " << testing_frames["pedestrian"]
      << " & " << testing_frames["bicyclist"]
      << " & " << testing_frames["background"]
      << " & " << total_testing_frames << " \\\\" << endl;
  oss << "\\hline" << endl;
  oss << "Total"
      << " & " << testing_frames["car"] + training_frames["car"]
      << " & " << testing_frames["pedestrian"] + training_frames["pedestrian"]
      << " & " << testing_frames["bicyclist"] + training_frames["bicyclist"]
      << " & " << testing_frames["background"] + training_frames["background"]
      << " & " << total_testing_frames + total_training_frames << " \\\\" << endl;

  oss << "\\hline" << endl;
  oss << "\\end{tabular}" << endl;

  return oss.str();
}

void labelAll(const string& label, const string& source, const string& destination) {
  TrackManager tm(source);
  for(size_t i = 0; i < tm.tracks_.size(); ++i)
    tm.tracks_[i]->label_ = label;

  tm.save(destination);
}

void labelAllCached(int label, const string& source, const string& destination) {
  TrackManagerCachedDescriptors tmcd(source);
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i)
    for(size_t j = 0; j < tmcd.tracks_[i]->frame_descriptors_.size(); ++j)
      tmcd.tracks_[i]->frame_descriptors_[j]->label_ = label;
	
  tmcd.save(destination);
}

void join(int argc, char** argv) {
    vector< shared_ptr<Track> > all;
    cout << "Saving to " << argv[2] << endl;
    cout << "Joining: " << endl;
    for(int i = 3; i < argc; ++i) {
      cout << " " << argv[i] << endl;

      TrackManager tm(argv[i]);
      all.reserve(all.size() + tm.tracks_.size() + 1);
      all.insert(all.end(), tm.tracks_.begin(), tm.tracks_.end());      
    }
    cout << endl;

    cout << "Saving..." << endl;
    TrackManager tm(all);
    tm.save(argv[2]);
}  

void split(const string& source_filename, const vector<double>& pcts, const vector<string>& files) {
  assert(pcts.size() == files.size());

  // -- Assert that the pcts add up to one.
  double sum = 0;
  for(size_t i = 0; i < pcts.size(); ++i)
    sum += pcts[i];
  assert(fabs(sum - 1) < 1e-6);

  // -- Load the source tracks.
  TrackManager src(source_filename);
  
  // -- Set up a place to distribute the tracks.
  vector< vector< shared_ptr<Track> > > output_tracks(files.size());
  vector<int> num_tracks(files.size());
  int num_distributed = 0;
  for(size_t i = 0; i < output_tracks.size(); ++i) {
    if(i == output_tracks.size() - 1)
      num_tracks[i] = src.tracks_.size() - num_distributed;
    else  {
      num_tracks[i] = pcts[i] * (double)src.tracks_.size();
      num_distributed += num_tracks[i];
    }

    output_tracks[i].reserve(num_tracks[i]);
  }

  // -- Distribute the source tracks.
  for(size_t i = 0; i < output_tracks.size(); ++i) {
    for(int j = 0; j < num_tracks[i]; ++j) {
      assert(src.tracks_.size() > 0);
      int idx = rand() % src.tracks_.size();
      output_tracks[i].push_back(src.tracks_[idx]);
      src.tracks_.erase(src.tracks_.begin() + idx);
    }
  }
  assert(src.tracks_.empty());

  // -- Save the distributed tracks.
  for(size_t i = 0; i < files.size(); ++i) {
    TrackManager tm(output_tracks[i]);
    tm.save(files[i]);
  }
}

void extractSeedLabels(TrackManager* src, int num, const string& seed_path, const string& remainder_path) {
  // -- Get the set of labels, ignoring background and unlabeled.
  set<string> labels;
  for(size_t i = 0; i < src->tracks_.size(); ++i) {
    string label = src->tracks_[i]->label_;
    if(label.compare("background") == 0 ||
       label.compare("unlabeled") == 0)
      continue;
    labels.insert(label);
  }

  // -- For each label, select num tracks.
  vector< shared_ptr<Track> > seed_tracks;
  set<string>::iterator it;
  for(it = labels.begin(); it != labels.end(); ++it) {
    string label = *it;

    for(int i = 0; i < num; ++i) {
      // -- Find a random track with the right label.
      string l;
      int idx;
      int counter = 0;
      while(true) {
	idx = rand() % src->tracks_.size();
	l = src->tracks_[idx]->label_;
	if(l.compare(label) == 0)
	  break;

	++counter;
	if(counter > 1e7) {
	  cerr << "No more objects with label " << label << "?  Aborting..." << endl;
	  assert(0);
	}
      }

      // -- Move it to the vector of selected tracks.
      seed_tracks.push_back(src->tracks_[idx]);
      src->tracks_.erase(src->tracks_.begin() + idx);
    }
  }

  // -- Save the seed and remainder tracks.
  TrackManager seed(seed_tracks);
  seed.save(seed_path);
  src->save(remainder_path);
}

int main(int argc, char** argv) {



  if(argc > 4 && strcmp(argv[1], "--join") == 0) {
    join(argc, argv);
  }
  //oss << " track_tools --latex-table --training TM [TM ...] --testing TM [TM ...]" << endl;
  else if(argc > 5 && strcmp(argv[1], "--latex-table") == 0) {
    vector<string> training_sets;
    vector<string> testing_sets;
    if(strcmp(argv[2], "--training") != 0) { 
      cout << usageString() << endl;
      return 1;
    }
	
    int i = 3;
    while(true) {
      if(strcmp(argv[i], "--testing") == 0)
	break;
      training_sets.push_back(argv[i]);
      ++i;
    }

    for(++i; i < argc; ++i) {
      testing_sets.push_back(argv[i]);
    }

    cout << "Training sets: " << endl;
    for(size_t i = 0; i < training_sets.size(); ++i)
      cout << training_sets[i] << endl;
    cout << endl;
    cout << "Testing sets: " << endl;
    for(size_t i = 0; i < testing_sets.size(); ++i)
      cout << testing_sets[i] << endl;
    cout << endl;

    if(training_sets.empty() || testing_sets.empty()) {
      cout << usageString() << endl;
      return 1;
    }

    cout << computeLatexTable(training_sets, testing_sets);
    
  }

  else if(argc > 2 && strcmp(argv[1], "--status") == 0) {
    vector<string> filenames;
    for(int i = 2; i < argc; ++i)
      filenames.push_back(argv[i]);
    status(filenames);
  }

  else if(argc > 4 && strcmp(argv[1], "--filter_only") == 0) {
    vector<string> labels;
    for(int i = 2; i < argc - 2; ++i)
      labels.push_back(argv[i]);

    string source(argv[argc - 2]);
    string destination(argv[argc - 1]);
    if(source.find(".tm") == string::npos || destination.find(".tm") == string::npos) {
      cout << "Source " << source << " or destination " << destination << " is not a .tm file!" << endl;
      return 1;
    }
      
    filter(labels, source, destination);
  }
    
  else if(argc == 6 && strcmp(argv[1], "--extract-seed-labels") == 0) {
    string seed_path(argv[4]);
    string remainder_path(argv[5]);
    int num = atoi(argv[2]);
    cout << "Extracting " << num << " labeled tracks of each (non-bg) class from " << argv[3] << ", saving seed labels into " << seed_path << " and the rest in " << remainder_path << endl;
    TrackManager src(argv[3]);

    extractSeedLabels(&src, num, seed_path, remainder_path);
  }
  
  else if(argc > 6 && strcmp(argv[1], "--split") == 0) {
    string source(argv[2]);
    vector<double> pcts;
    vector<string> files;
    assert(argc % 2 == 1);
    for(int i = 3; i < argc; i+=2) {
      pcts.push_back(atof(argv[i]));
      files.push_back(argv[i+1]);
    }
    split(source, pcts, files);
  }
  
  else if(argc > 4 && strcmp(argv[1], "--evensplit") == 0) {
    int num = argc - 3;
    cout << "Randomly distributing tracks from " << argv[2] << " into " << num << " parts." << endl;

    cout << "Loading " << argv[2] << endl;
    TrackManager tm(argv[2]);
    vector< shared_ptr<Track> > src = tm.tracks_;
    size_t tracks_per_dest = floor((float)src.size() / (float)num);
    
    for(int i = 3; i < argc; ++i) {
      cout << "Generating " << argv[i] << endl;
      vector< shared_ptr<Track> > dest;
      dest.reserve(tracks_per_dest);

      if(i == argc - 1)
	dest = src;
      else { 
	for(size_t j = 0; j < tracks_per_dest; ++j) {
	  int idx = rand() % src.size();
	  dest.push_back(*(src.begin() + idx));
	  src.erase(src.begin() + idx);
	}
      }

      cout << "Saving " << argv[i] << endl;
      TrackManager tm2(dest);
      tm2.save(argv[i]);
    }
  }

  else if(argc == 5 && strcmp(argv[1], "--label-all") == 0) {
    cout << "Labeling all tracks from " << argv[3] << " as " << argv[2] << ", saving in " << argv[4] << endl;
    labelAll(argv[2], argv[3], argv[4]);
  }

  else if(argc == 5 && strcmp(argv[1], "--label-all-cached") == 0) {
    cout << "Labeling all tracks from " << argv[3] << " as " << atoi(argv[2]) << ", saving in " << argv[4] << endl;
    labelAllCached(atoi(argv[2]), argv[3], argv[4]);
  }
  
  else if(argc == 4 && strcmp(argv[1], "--cache-descriptors") == 0) {
    cout << "Computing descriptors for " << argv[2] << ", caching them in " << argv[3] << endl;
    cacheDescriptors(TrackManager(argv[2]), argv[3]);
  }

  else if(argc > 3 && strcmp(argv[1], "--join-tm-mbd") == 0) {
    bool allow_unlabeled = true;
    int argstart = 2;
    if(strcmp(argv[2], "--no-unlabeled") == 0) { 
      allow_unlabeled = false;
      argstart = 3;
    }
    
    vector<string> input_paths;
    string output_path = argv[argc - 1];
    cout << "Converting" << endl;
    for(int i = argstart; i < argc - 1; ++i) {
      input_paths.push_back(argv[i]);
      cout << " " << argv[i] << endl;
    }
    cout << "into" << endl << " " << output_path << endl;

    joinCachedDescriptors(input_paths, output_path, allow_unlabeled);
  }

  else if(argc > 2 && strcmp(argv[1], "--check-for-duplicates") == 0) {
    cout << "Checking for duplicates." << endl;
    vector<string> paths;
    for(int i = 2; i < argc; ++i)
      paths.push_back(argv[i]);
    checkForDuplicates(paths);
  }

  else if(argc == 4 && strcmp(argv[1], "--label-diff") == 0) {
    cout << "Diffing labels." << endl;
    labelDiff(argv[2], argv[3]);
  }

  else {
    cout << usageString() << endl;
  }

  
  
  return 0;
}
