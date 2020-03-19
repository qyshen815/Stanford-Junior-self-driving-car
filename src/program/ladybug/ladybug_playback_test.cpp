#include <roadrunner.h>
#include <ladybug_playback.h>
#include <image.h>

using namespace vlr;

int 
main(int argc, char **argv)
{

  if (argc!=2) {
    fprintf( stderr, "usage: %s <LLF-FILE>\n", argv[0] );
    exit(0);
  }

  LadybugPlayback *lp = new LadybugPlayback;

  lp->openLLF(argv[1]);
  lp->readTimestampPacket(1267833486.5);
  dgc_image_t *img = lp->cameraImage(1);

  dgc_image_write(img, "out.png");

  return 0;
}
