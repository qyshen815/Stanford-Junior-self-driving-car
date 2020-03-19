#include <aw_roadNetwork.h>
#include <aw_roadNetworkSearch.h>
#include <rndfgl.h>
#include <iostream>
#include <iomanip>
#include <imagery.h>
#include <string>

#include "conout.h"
#include "REConsole.h"
#include "rndf_edit_gui.h"

#include <QtGui/QtGui>
#include <QtGui/QApplication>

using namespace vlr;

namespace vlr {

QApplication* qtapp=NULL;
RNDFEditGUI* gui=NULL;

//typedef void (* event_callback) (event ev, ios_base& ios, int index);

class BufferedStringBuf: public std::streambuf {
public:
    BufferedStringBuf(unsigned int bufSize = 256) {
        if (bufSize) {
            char *ptr = new char[bufSize];
            setp(ptr, ptr + bufSize);
        }
        else {
            setp(0, 0);
        }
    }

    virtual ~BufferedStringBuf() {
        sync();
        delete[] pbase();
    }

    virtual void writeString(const std::string &str) {
        conout(str.c_str());
    }

private:
    int overflow(int c) {
        sync();

        if (c != EOF) {
            if (pbase() == epptr()) {
                std::string temp;

                temp += char(c);
                writeString(temp);
            }
            else {
                sputc(c);
            }
        }

        return 0;
    }

    int sync() {
        if (pbase() != pptr()) {
            int len = int(pptr() - pbase());
            std::string temp(pbase(), len);
            writeString(temp);
            setp(pbase(), epptr());
        }
        return 0;
    }
};

} // namespace vlr

using namespace vlr;

int main(int argc, char **argv) {
  //param_struct_t param;
  //kogmo_param_file_t* params=NULL;
  //
  if (argc <= 1) {
    std::cout << "Error: Missing RNDF filename.\n";
    std::cout << argv[0] << " rndf_filename [imagery_path]\n";
    std::cout << "If no imagery path is given, imagery is read from $VLR_ROOT/data/imagery.\n";
    char* race_home = getenv("VLR_ROOT");
    if (race_home) {
      std::cout << "Currently $VLR_ROOT points to " << race_home << std::endl;
    }
    else {
      std::cout << "Currently $VLR_ROOT is not set :-(\n";
    }
    exit(0);
  }
  glutInit(&argc, argv);

  std::string imagery_folder;

  if (argc > 2) {
    imagery_folder = argv[2];
  }
  else {
    std::string race_dir;
    char* race_home = getenv("VLR_ROOT");
    if (race_home) {
      race_dir = race_home;
    }
    else {
      race_dir = "~";
    }

    imagery_folder = race_dir + "/data/imagery";
  }

  qtapp = new QApplication(argc, argv);
  qtapp->connect(qtapp, SIGNAL(lastWindowClosed()), qtapp, SLOT(quit()));

  int imagery_zoom_level = 10; //kogmo_params_get_param_int(params, "imagery", "zoom_level");
  std::string rndf_name(argv[1]);

  gui = new RNDFEditGUI(rndf_name, imagery_folder, imagery_zoom_level, 3, 3);

  gui->show();

  BufferedStringBuf sbuf;
  std::streambuf* old_buf = std::cout.rdbuf(&sbuf);

  // ...and loop (QT)
  qtapp->exec();

  delete gui;
  std::cout.rdbuf(old_buf);

  return 0;
}
