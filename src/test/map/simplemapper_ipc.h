#ifndef DGC_MAPPER_IPC_H
#define DGC_MAPPER_IPC_H

#ifdef __cplusplus
extern "C" {
#endif

void
mapper_register_ipc_messages(void);

void
mapper_publish_diff(dgc_grid_p grid, double min_x, double min_y,
		    double max_x, double max_y, int use_compression,
		    int whole_map);

#ifdef __cplusplus
}
#endif

#endif
