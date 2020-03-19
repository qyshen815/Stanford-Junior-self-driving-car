#ifndef DGC_TEXTURECACHE_H
#define DGC_TEXTURECACHE_H

#include <roadrunner.h>
#include <textures.h>
#include "imagery.h"

namespace vlr {

typedef enum { UNINITIALIZED, REQUESTED, READY } dgc_image_state;

void dgc_texture_cache_initialize();

int dgc_texture_cache_sync();

dgc_gl_texture_t* dgc_texture_cache_get(const char* imagery_root, char *detected_subdir, image_tile_id id, int priority, dgc_image_state* state);

void dgc_texture_cache_stop();

int dgc_texture_cache_last_grayscale();

void dgc_texture_cache_set_version(int version_num);

int dgc_texture_cache_get_version();

} // namespace vlr

#endif
