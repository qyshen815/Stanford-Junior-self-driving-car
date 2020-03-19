#include <roadrunner.h>
#include <perception_interface.h>
#include <heartbeat_interface.h>
#include <grid.h>
#include "simplemapper.h"

void mapper_register_ipc_messages(void)
{
  dgc_ipc_define_test_exit(DGC_PERCEPTION_STATICDIFF_NAME, 
			   DGC_PERCEPTION_STATICDIFF_FMT);
  dgc_ipc_define_test_exit(DGC_HEARTBEAT_NAME, DGC_HEARTBEAT_FMT);
}

void mapper_publish_diff(dgc_grid_p grid, double min_x, double min_y,
			 double max_x, double max_y, int use_compression,
			 int whole_map)
{
  static int first = 1;
  static dgc_perception_staticdiff_message msg;
  static unsigned char *offset = NULL;
  static int num_offsets = 0, max_offsets = 0;
  static unsigned char *data = NULL;
  static int num_data = 0, max_data = 0;

  int num_changes, o, last_o, next_offset, r, c, r1, c1, r2, c2;
  IPC_RETURN_TYPE err;
  map_cell_p cell;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }

  use_compression = 0;

  msg.total_map_width = grid->cols;
  msg.total_map_height = grid->rows;
  msg.resolution = grid->resolution;

  if(whole_map) {
    r1 = 0;
    c1 = 0;
    r2 = grid->rows - 1;
    c2 = grid->cols - 1;
  }
  else {
    dgc_grid_xy_to_rc_local(grid, min_x, min_y, &r1, &c1);
    dgc_grid_xy_to_rc_local(grid, max_x, max_y, &r2, &c2);
  }
  msg.change_origin_x = grid->map_c0 + c1;
  msg.change_origin_y = grid->map_r0 + r1;
  msg.change_width = c2 - c1 + 1;
  last_o = 0;

  num_offsets = 0;
  num_data = 0;
  num_changes = 0;

  for(r = r1; r <= r2; r++)
    for(c = c1; c <= c2; c++) {
      /* lookup the cell */
      cell = (map_cell_p)dgc_grid_get_rc_local(grid, r, c);

      /* ignore NULL cells and unchanged cells */
      if(cell == NULL || !cell->changed)
	continue;

      o = (r - r1) * msg.change_width + (c - c1);
      next_offset = o - last_o;

      /* fill in offset */
      while(next_offset >= 255) {
	if(num_offsets >= max_offsets) {
	  max_offsets += 10000;
	  offset = (unsigned char *)realloc(offset, max_offsets);
	  dgc_test_alloc(offset);
	}
	offset[num_offsets] = 255;
	next_offset -= 255;
	num_offsets++;
      }
      if(num_offsets >= max_offsets) {
	max_offsets += 10000;
	offset = (unsigned char *)realloc(offset, max_offsets);
	dgc_test_alloc(offset);
      }
      offset[num_offsets] = next_offset;
      num_offsets++;

      /* fill in data */
      if(num_data >= max_data) {
	max_data += 10000;
	data = (unsigned char *)realloc(data, max_data);
	dgc_test_alloc(data);
      }
      if(cell->unknown)
	data[num_data] = 0;
      else
	data[num_data] = 1 + (int)dgc_clamp(cell->value * 254, 0, 254);
      num_data++;

      last_o = o;

      /* unmark the cell as changed */
      cell->changed = 0;
      num_changes++;
    }

  /* don't publish a message if there are no changes */
  if(num_changes == 0)
    return;

  msg.compressed = 0;
  msg.num_changes = num_changes;
  msg.offset_bytes = num_offsets;
  msg.offset = offset;
  msg.data_bytes = num_data;
  msg.data = data;
  msg.timestamp = dgc_get_time();

  err = IPC_publishData(DGC_PERCEPTION_STATICDIFF_NAME, &msg);
  dgc_test_ipc_exit(err, "Could not publish", DGC_PERCEPTION_STATICDIFF_NAME);
  fprintf(stderr, "P");
}
