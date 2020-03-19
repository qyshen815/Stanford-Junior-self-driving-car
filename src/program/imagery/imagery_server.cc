#include <roadrunner.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>
#include <sockutils.h>
#include "imagery.h"
#include "imagery_proj.h"

#define        DEFAULT_PORT          3000

using namespace vlr;

CURL *curl_handle = NULL;
pthread_mutex_t curl_mutex = PTHREAD_MUTEX_INITIALIZER;

int start_server(unsigned short portnum)
{
  struct sockaddr_in servaddr;
  int arg = 1, sockfd;
   
  if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    return(-1);

  memset(&servaddr, 0, sizeof(struct sockaddr_in)); 
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(portnum); 
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void *)&arg, sizeof(arg));

  if(bind(sockfd, (struct sockaddr *)&servaddr, 
	  sizeof(struct sockaddr_in)) < 0) {
    close(sockfd);
    return(-1);
  }
  listen(sockfd, 5);
  return(sockfd);
}

void shutdown_handler(int x)
{
  if(x == SIGPIPE)
    ;
  else
    while(waitpid(-1, NULL, WNOHANG) > 0);
}

void parse_command(unsigned char *command, image_tile_id *imagery_command)
{
  imagery_command->type = command[0];
  imagery_command->x = *((int *)(command + 1));
  imagery_command->y = *((int *)(command + 5));
  imagery_command->res = *((int *)(command + 9));
  imagery_command->zone = command[13];
  if(imagery_command->type == DGC_IMAGERY_TYPE_LASER)
    imagery_command->zone_letter = command[14];
}

int fetch_url(CURL *curl_handle, char *url, char *filename)
{
  FILE *out_fp;
  int err;

  out_fp = fopen(filename, "w");
  if(out_fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for writing.\n", filename);
    return -1;
  }
  curl_easy_setopt(curl_handle, CURLOPT_URL, url);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)out_fp);
  curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
  err = curl_easy_perform(curl_handle);
  fclose(out_fp);
  if(err != 0)
    return -1;
  return 0;
}

void fetch_terraserver(CURL *curl_handle, image_tile_id id, char *filename)
{
  char dirname[1000], url[1000];
  int terra_num = 4;

  if(id.type == DGC_IMAGERY_TYPE_TOPO) {
    dgc_terra_topo_tile_filename(id, filename, 1);
    terra_num = 2;
  }
  else if(id.type == DGC_IMAGERY_TYPE_COLOR) {
    dgc_terra_color_tile_filename(id, filename, 1);
    terra_num = 4;
  }
  else if(id.type == DGC_IMAGERY_TYPE_BW) {
    dgc_terra_bw_tile_filename(id, filename);
    terra_num = 1;
  }

  if(dgc_file_exists(filename)) 
    return;
     
  strcpy(dirname, "usgs");
  if(!dgc_file_exists(dirname)) 
    mkdir(dirname, 0755);
  sprintf(dirname, "usgs/%d", terra_num);
  if(!dgc_file_exists(dirname)) 
    mkdir(dirname, 0755);
  sprintf(dirname, "usgs/%d/%d", terra_num, id.res);
  if(!dgc_file_exists(dirname)) 
    mkdir(dirname, 0755);
  sprintf(dirname, "usgs/%d/%d/%d", terra_num, id.res,
          id.x);
  if(!dgc_file_exists(dirname)) 
    mkdir(dirname, 0755);
  sprintf(url,
          "http://terraserver-usa.com/tile.ashx?T=%d&S=%d&X=%d&Y=%d&Z=%d", 
          terra_num, id.res, id.x, id.y, id.zone);

  fprintf(stderr, "Fetched %s\n", url);
  fetch_url(curl_handle, url, filename);
}

void serve_files(int sock)
{
  unsigned char command[1000], buffer[100001], filename[1000];
  image_tile_id imagery_command;
  int n, nread, nwritten;
  off64_t file_size;
  FILE *fp;

  do {
    /* read command */
    if(dgc_sock_readn(sock, command, 15, -1) < 15)
      return;
    parse_command(command, &imagery_command);

    if(imagery_command.type == DGC_IMAGERY_TYPE_COLOR ||
       imagery_command.type == DGC_IMAGERY_TYPE_TOPO ||
       imagery_command.type == DGC_IMAGERY_TYPE_BW) {
      pthread_mutex_lock(&curl_mutex);
      fetch_terraserver(curl_handle, imagery_command, (char *)filename);
      pthread_mutex_unlock(&curl_mutex);
    }
    else if(imagery_command.type == DGC_IMAGERY_TYPE_LASER) 
      dgc_laser_tile_filename(imagery_command, (char *)filename, 1);
    else if(imagery_command.type == DGC_IMAGERY_TYPE_GSAT) 
      dgc_gmaps_tile_filename(imagery_command, (char *)filename);
    else if(imagery_command.type == DGC_IMAGERY_TYPE_DARPA) 
      dgc_darpa_tile_filename(imagery_command, (char *)filename, 1);
    else
      dgc_die("Error: unknown command\n");
    
    /* if the file doesn't exist, return 0 length */
    if(!dgc_file_exists((char *)filename)) {
      file_size = 0;
      nwritten = dgc_sock_writen(sock, &file_size, sizeof(off64_t), -1);
      if(nwritten < 0) 
	return;
    }
    else {
      /* write file size */
      file_size = dgc_file_size((char *)filename);
      nwritten = dgc_sock_writen(sock, &file_size, sizeof(off64_t), -1);
      if(nwritten < 0) 
	return;
      
      /* write file contents */
      fp = fopen((char *)filename, "r");
      while(file_size > 0) {
	n = 100000;
	if(n > file_size)
	  n = file_size;
	
	nread = fread(buffer, n, 1, fp);
	if(nread == 0) {
	  fclose(fp);
	  return;
	}
	
	nwritten = dgc_sock_writen(sock, buffer, n, -1);
	if(nwritten < 0) {
	  fclose(fp);
	  return;
	}
	file_size -= n;
      }
      fclose(fp);
    }
  } while(1);
}

int main(int argc, char **argv)
{ 
  int s, t, portnum = DEFAULT_PORT;

  if(argc >= 2)
    portnum = atoi(argv[1]);

  fprintf(stderr, "Starting imagery server on port %d.\n", portnum);

  curl_handle = curl_easy_init();
  
  if((s = start_server(portnum)) < 0) 
    dgc_die("Error: could not start server at port %d\n", portnum);

  /* this eliminates zombies */
  signal(SIGCHLD, shutdown_handler);
  signal(SIGPIPE, shutdown_handler);
  //  daemon(0, 0);
  
  while(1) {
    if((t = accept(s, NULL, NULL)) < 0) { 
      if(errno == EINTR)             /* EINTR might happen on accept(), */
	continue;                    /* try again */
      perror("accept");                   
      exit(1);
    }

    switch(fork()) {                  
    case -1:                          
      perror("fork");
      close(s);
      close(t);
      exit(1);
    case 0:                           /* child */
      close(s);
      serve_files(t);
      exit(0);
    default:                          /* parent */
      close(t);                       
      continue;
    }
  }
}

