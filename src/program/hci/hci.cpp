#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <error_interface.h>
#include <hci_interface.h>
#include <param_interface.h>

#include <SDL.h>
#include <SDL_mixer.h>

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace dgc;

using std::vector;
using std::map;
using std::string;
using std::istringstream;

/* output sampling freq, this is based on the .ogg files */
const int FREQ = 16000;

/* stereo output */
const int CHANNELS = 2;

/* mix chunk size */
const int CHUNK_SIZE = 1024;

/* map for sound files and wait times */
typedef map <string, Mix_Music *> SoundMap;
typedef vector <Mix_Music *> SoundVec;

/* variables */
static SoundMap soundMap;

static pthread_mutex_t audio_mutex = PTHREAD_MUTEX_INITIALIZER;
SoundVec next_clip, current_clip;
int unplayed_audio = 0;

/* parameters */
char *sounddir = NULL;

/* close audio and cleanup */

void close_audio(void)
{
  SoundMap::iterator iter;

  Mix_CloseAudio();
  for(iter = soundMap.begin(); iter != soundMap.end(); iter++) {
    Mix_FreeMusic(iter->second);
    soundMap[iter->first] = NULL;
  }
}

/* initialize audio system, load sound clips */

void init_audio(char *sounddir)
{
  char *filename, *err, line[1001], key[1000], clip[1000];
  char sound_filename[1000];
  FILE *fp;

  /* look for config file */
  filename = (char *)calloc(strlen(sounddir) + 50, 1);
  dgc_test_alloc(filename);
  strcpy(filename, sounddir);
  strcat(filename, "/sound-config.txt");
  
  /* init sdl for sound */
  SDL_Init(SDL_INIT_AUDIO);
  
  /* open audio device, frequency is based on ogg files */
  if(Mix_OpenAudio(FREQ, MIX_DEFAULT_FORMAT, CHANNELS, CHUNK_SIZE) != 0) 
    fprintf(stderr, "Failed to initialize audio: %s\n", Mix_GetError());
  else
    atexit(close_audio);     /* close audio when exiting */

  fp = fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);
  do {
    err = fgets(line, 1000, fp);
    if(err != NULL && strlen(line) > 2) {
      sscanf(line, "%s %s", key, clip);
      
      if(key[0] == '#' || strlen(key) == 0 || strlen(clip) == 0)
	continue;
      
      strcpy(sound_filename, sounddir);
      strcat(sound_filename, "/");
      strcat(sound_filename, clip);

      if(soundMap[key]) 
	fprintf(stderr, "Duplicate Key: %s %s\n", key, sound_filename);
      else 
	soundMap[key] = Mix_LoadMUS(sound_filename);
      if(!soundMap[key]) 
	fprintf(stderr, "Failed to load: %s %s\n", key, sound_filename);
    }
  } while(err != NULL);
  fclose(fp);
  free(filename);
  fprintf(stderr, "Successfully loaded config file.\n");
}

/* speak a vector of sound clips */

void speak(SoundVec& clips)
{
  SoundVec::iterator iter;

  for(iter = clips.begin(); iter != clips.end(); iter++) {
    Mix_PlayMusic(*iter, 0);
    while(Mix_PlayingMusic())
      SDL_Delay(100);
  }
}

void *audio_thread(__attribute__ ((unused)) void *ptr)
{
  init_audio(sounddir);
  while(1) {
    if(unplayed_audio) {
      pthread_mutex_lock(&audio_mutex);
      current_clip = next_clip;
      next_clip.clear();
      unplayed_audio = 0;
      pthread_mutex_unlock(&audio_mutex);
      speak(current_clip);
    }
    usleep(100000);
  }
  return NULL;
}

void hci_add_string(char *str)
{
  if(str == NULL)
    return;
  
  // tokenize msg
  SoundVec clips;
  istringstream is;
  is.str(str);
  string tmp;
  
  while(is >> tmp) {
    // test if token is a number
    if(tmp.find_first_not_of("1234567890") == string::npos) {
      // deal with numbers one digit at a time
      for(string::size_type i = 0; i < tmp.size(); i++) {
	Mix_Music *clip = soundMap[tmp.substr(i, 1)];
	if(clip) 
	  clips.push_back(clip);
      }
    } 
    else { // lookup entire token
      Mix_Music *clip = soundMap[tmp];
      if(clip)
	clips.push_back(clip);
    }
  }

  if(!clips.empty()) {
    pthread_mutex_lock(&audio_mutex);
    next_clip = clips;
    unplayed_audio = 1;
    pthread_mutex_unlock(&audio_mutex);
    fprintf(stderr, "Received Msg: %s\n", str);
  }
}

void error_string_handler(ErrorString *error)
{
  hci_add_string(error->string);
}

void hci_audio_handler(HciAudio *hci)
{   
  hci_add_string(hci->msg);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param param[] = {
    {"hci", "sounddir", DGC_PARAM_FILENAME, &sounddir, 0, NULL},
  };
  pint->InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc = NULL;
  ParamInterface *pint;
  pthread_t thread;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->ConnectLocked("hci") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);

  pthread_create(&thread, NULL, audio_thread, NULL);

  ipc->Subscribe(ErrorStringID, error_string_handler, DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(HciAudioID, hci_audio_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
