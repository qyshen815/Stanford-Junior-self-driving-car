#include <roadrunner.h>
#include <logio.h>

using namespace dgc;

dgc_FILE *infile = NULL;

typedef struct {
  char *name;
  int count;
  long int total_bytes;
  double last_log_timestamp;
  double largest_logts_gap;

  double last_timestamp;
  double largest_ts_gap;
} message_info;

int num_messages = 0;
message_info *message = NULL;

int compare_messages(const void *a, const void *b)
{
  message_info *m1 = (message_info *)a, *m2 = (message_info *)b;
  
  if(m1->total_bytes < m2->total_bytes)
    return 1;
  else if(m1->total_bytes == m2->total_bytes)
    return 0;
  else 
    return -1;
}

int main(int argc, char **argv)
{
  LineBuffer *line_buffer = NULL;
  double log_timestamp = 0, timestamp = 0;
  long int total = 0, sum = 0;
  double final_time = 0;
  int i, j, l, found;
  char *line = NULL;
  char name[1000];
  char *mark;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s logfile-name\n", argv[0]);
  
  infile = dgc_fopen(argv[1], "r");
  if(infile == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(infile);

    if(line != NULL) {
      /* extract log timestamp */
      if(line[0] != '#') {
	mark = line + strlen(line) - 1;
	while(mark[0] != ' ')
	  mark--;
	log_timestamp = atof(mark);
	final_time = log_timestamp;
	
	/* walk back two words */
	mark--;
	while(mark[0] != ' ')
	  mark--;
	mark--;
	while(mark[0] != ' ')
	  mark--;
	
	timestamp = atof(mark);
      }
      
      /* extract message name */
      j = 0;
      l = strlen(line);
      while(j < l && line[j] != ' ') {
	name[j] = line[j];
	j++;
      }
      name[j] = '\0';
    
      /* look for it in the list */
      found = 0;
      for(j = 0; j < num_messages; j++)
	if(strcmp(name, message[j].name) == 0) {
	  message[j].total_bytes += l;
	  message[j].count++;
	  
	  if(log_timestamp - message[j].last_log_timestamp > 3)
	    fprintf(stderr, "Gap of %f at %f seconds %s\n",
		    log_timestamp - message[j].last_log_timestamp,
		    log_timestamp, message[j].name);
	  
	  if(log_timestamp - message[j].last_log_timestamp > 
	     message[j].largest_logts_gap)
	    message[j].largest_logts_gap = log_timestamp - 
	      message[j].last_log_timestamp;
	  message[j].last_log_timestamp = log_timestamp;
	  
	  if(timestamp - message[j].last_timestamp > message[j].largest_ts_gap)
	    message[j].largest_ts_gap = timestamp - message[j].last_timestamp;
	  message[j].last_timestamp = timestamp;
	  found = 1;
	  break;
	}
      
      /* if not found, add it to the list */
      if(!found) {
	num_messages++;
	message = (message_info *)realloc(message, num_messages * 
					  sizeof(message_info));
	dgc_test_alloc(message);
	message[num_messages - 1].name = (char *)calloc(l + 1, 1);
	dgc_test_alloc(message[num_messages - 1].name);
	strcpy(message[num_messages - 1].name, name);
	message[num_messages - 1].total_bytes = l;
	message[num_messages - 1].count = 1;
	message[num_messages - 1].last_log_timestamp = log_timestamp;
	message[num_messages - 1].last_timestamp = timestamp;
	message[num_messages - 1].largest_logts_gap = 0;
	message[num_messages - 1].largest_ts_gap = 0;
      }
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");
  dgc_fclose(infile);
    
  for(i = 0; i < num_messages; i++)
    total += message[i].total_bytes;

  qsort(message, num_messages, sizeof(message_info), compare_messages);

  fprintf(stderr, "\nLogfile is %ld bytes long (uncompressed) and represents %.2f seconds of data\n\n", total, final_time);
  fprintf(stderr, //"%30s %10s   %6s     %6s\n",
          "%30s %10s %10s   %6s      %6s    %6s\n", 
          "MESSAGE NAME", "BYTES", "COUNT", "FREQ", "PERCENT", "CUMUL");
  fprintf(stderr, "------------------------------------------------------------------------------------\n");
  for(i = 0; i < num_messages; i++) {
    sum += message[i].total_bytes;
    fprintf(stderr, "%30s %10ld %10d   %5.1f Hz   %6.2f%%    %6.2f%%\n", 
            message[i].name, message[i].total_bytes, message[i].count, 
            message[i].count / final_time,
            message[i].total_bytes / (double)total *
            100.0, sum / (double)total * 100.0);
  }

  fprintf(stderr, "\n%30s %15s %15s\n", "MESSAGE_NAME", "TS GAP (ms)", 
          "LOG TS GAP (ms)");
  fprintf(stderr, "--------------------------------------------------------------\n");
  for(i = 0; i < num_messages; i++) {
    fprintf(stderr, "%30s %15.1f %15.1f\n", message[i].name, 
            message[i].largest_ts_gap * 1000.0,
            message[i].largest_logts_gap * 1000.0);
  }
  return 0;
}
