#ifndef DGC_SIMPLEMAPPER_H
#define DGC_SIMPLEMAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  char changed, unknown;
  float value;
} map_cell_t, *map_cell_p;

#ifdef __cplusplus
}
#endif

#endif
