#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>

using namespace dgc;
using namespace vlr;

IpcInterface *ipc;

void applanix_handler( ApplanixPose *pose )
{
  double current_time;
  static double last_time = 0;
  double delay;
  double spacing;
  static double avg_delay = 0;
  static double max_delay = 0;
  static double last_print_time = 0;
  static double avg_spacing = 0;
  static double last_delay_time = 0;
  static long delay_count = 0;
  static long last_count = 0;
  static long total_count = 0;
  static long last_total = 0;
  static double avg_bunched = 0;
  static int max_bunched = 0;
  static double bunch_count = 0;

  static long avg_delay_count = 0;
  static long num_bunches = 0;

  current_time = dgc_get_time();
 
  if(last_time == 0) {
    last_time = current_time;
    last_print_time = current_time;
    last_delay_time = current_time;
    return;
  }

  total_count++;

  delay = current_time - last_time;

  if(delay > 0.01) {
    avg_delay_count++;
    spacing = current_time - last_delay_time;
    avg_spacing = avg_spacing + 1.0 / avg_delay_count * (spacing - avg_spacing);
    avg_delay = avg_delay + 1.0 / avg_delay_count * (delay - avg_delay);
    last_delay_time = current_time;
  }

  if(delay > max_delay) {
    max_delay = delay;
  }

  if(delay < 0.001) {
    bunch_count++;
    delay_count++;
  }  
  else if(bunch_count > 0) {
    if(bunch_count + 1 > max_bunched) {
      max_bunched = bunch_count + 1;
    }
    num_bunches++;
    avg_bunched = avg_bunched + 1.0 / num_bunches * (bunch_count + 1 - avg_bunched);
    bunch_count = 0;
  }

  if(current_time > last_print_time + 1) {
    printf("Total packets: %ld\n", total_count);
    printf("Total missed packets: %ld\n", delay_count);
    printf("Packets/s: %ld\n", total_count - last_total);
    printf("Missed packets last second: %ld\n", delay_count - last_count);
    printf("Average bunch: %.3f\n", avg_bunched);
    printf("Max bunch: %d\n", max_bunched);
    printf("Average delay: %.3f\n", avg_delay);
    printf("Max delay: %.3f\n", max_delay);
    printf("Average spacing: %.3f\n\n", avg_spacing);
    last_count = delay_count;
    last_print_time = current_time;
    last_total = total_count;
  }

  last_time = current_time;
}

int main(int /*argc*/, char **argv)
{
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(ApplanixPoseID, applanix_handler);

  ipc->Dispatch();
  return 0;
}
