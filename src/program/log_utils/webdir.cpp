#define  __USE_LARGEFILE64

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <roadrunner.h>

#define _GNU_SOURCE
#include <fnmatch.h>

#include "webdir.h"


#define MAX_NUM_FILES             1024
#define MAX_NAME_LENGTH            256

#define FALSE                      0
#define TRUE                       1
#define FLAG_FILE_NAME             ".dir-recreate-flag"
#define ICON_DIR                   ".icons"

#define NUM_DIR_ICON_PER_ROW       4
#define NUM_FILE_ICON_PER_ROW      2
#define MIN_VIDEO_SIZE             (6 * 1024)
#define ICON_WIDTH                 86
#define ICON_HEIGHT                64

#define ANIMATED_GIF_DIR             ".animated-gif"
#define ANIMATED_GIF_LENGTH_IN_SEC   25.0
#define ANIMATED_GIF_FRAME_RATE      1.0

int     recursive  = FALSE;
int     use_ffmpeg = TRUE;
int     force      = FALSE;
int     googledir  = TRUE;

#define ICON_UP_NAME               "up.png"
#define ICON_FOLDER_NAME           "folder.png"
#define ICON_DOCUMENT_NAME         "document.png"
#define ICON_IMAGE_NAME            "image.png"
#define ICON_PDF_NAME              "pdf.png"
#define ICON_POSTSCRIPT_NAME       "postscript.png"
#define ICON_VIDEO_NAME            "video.png"
#define ICON_SOUND_NAME            "sound.png"
#define ICON_SOURCE_NAME           "source.png"
#define ICON_SOURCE_C_NAME         "source_c.png"
#define ICON_SOURCE_CPP_NAME       "source_cpp.png"
#define ICON_SOURCE_H_NAME         "source_h.png"
#define ICON_SOURCE_JAVA_NAME      "source_java.png"
#define ICON_SOURCE_MOC_NAME       "source_moc.png"
#define ICON_SOURCE_O_NAME         "source_o.png"
#define ICON_VELODYNE_NAME         "velodyne.png"
#define ICON_JUNIOR_NAME           "junior.png"
#define ICON_GOOGLE_OVERLAY_NAME   "google_overlay.png"
#define ICON_GOOGLE_MAPS_NAME      "google_maps.png"
#define ICON_LADYBUG_NAME          "ladybug.png"
#define ICON_BOSCHCAM_NAME         "boschcam.png"

#define NUM_ICONS  21

char          * icon_names[] = { ICON_UP_NAME, ICON_FOLDER_NAME, ICON_DOCUMENT_NAME, ICON_IMAGE_NAME, ICON_PDF_NAME, ICON_POSTSCRIPT_NAME, ICON_VIDEO_NAME, ICON_SOUND_NAME, ICON_SOURCE_NAME, ICON_SOURCE_C_NAME, ICON_SOURCE_CPP_NAME, ICON_SOURCE_H_NAME, ICON_SOURCE_JAVA_NAME, ICON_SOURCE_MOC_NAME, ICON_SOURCE_O_NAME, ICON_VELODYNE_NAME, ICON_JUNIOR_NAME, ICON_GOOGLE_OVERLAY_NAME, ICON_GOOGLE_MAPS_NAME, ICON_LADYBUG_NAME, ICON_BOSCHCAM_NAME };

int             icon_sizes[] = { ICON_UP_SIZE, ICON_FOLDER_SIZE, ICON_DOCUMENT_SIZE, ICON_IMAGE_SIZE, ICON_PDF_SIZE, ICON_POSTSCRIPT_SIZE, ICON_VIDEO_SIZE, ICON_SOUND_SIZE, ICON_SOURCE_SIZE, ICON_SOURCE_C_SIZE, ICON_SOURCE_CPP_SIZE, ICON_SOURCE_H_SIZE, ICON_SOURCE_JAVA_SIZE, ICON_SOURCE_MOC_SIZE, ICON_SOURCE_O_SIZE, ICON_VELODYNE_SIZE, ICON_JUNIOR_SIZE, ICON_GOOGLE_OVERLAY_SIZE, ICON_GOOGLE_MAPS_SIZE, ICON_LADYBUG_SIZE, ICON_BOSCHCAM_SIZE };

unsigned char * icon_data[]  = { icon_up, icon_folder, icon_document, icon_image, icon_pdf, icon_postscript, icon_video, icon_sound, icon_source, icon_source_c, icon_source_cpp, icon_source_h, icon_source_java, icon_source_moc, icon_source_o, icon_velodyne, icon_junior, icon_google_overlay, icon_google_maps, icon_ladybug, icon_boschcam };

  void
create_icons( void )
{
  FILE  * fp;
  int     i, j;
  char    filename[MAX_NAME_LENGTH];

  for (j=0; j<NUM_ICONS; j++) {
    snprintf( filename, MAX_NAME_LENGTH, "%s/%s", ICON_DIR, icon_names[j] );
    if ((fp=fopen(filename,"w"))==0) {
      fprintf( stderr, "ERROR: can't write icon file %s !\n", filename );
      exit(0);
    } else {
      for (i=0; i<icon_sizes[j]; i++) {
        fputc( icon_data[j][i], fp ); 
      }
      fclose(fp);
    }
  }

}

  char *
uppath( int level )
{
  char  * path;
  int     i;
  path = (char *) malloc( level*3+2 * sizeof( char ) );
  path[0] = 0;
  for (i=0; i<level; i++) {
    path = strncat( path, "../", MAX_NAME_LENGTH );
  }
  return(path);
}

  int
file_exists(char *filename)
{
  FILE *fp;

  fp = fopen(filename, "r");
  if(fp == NULL)
    return 0;
  else {
    fclose(fp);
    return 1;
  }
}

  void
create_page_header( FILE * fp, int level __attribute__((unused)) )
{
  fprintf( fp, "<HTML>\n" );
  fprintf( fp, "<HEADER>\n" );
  fprintf( fp, "</HEADER>\n" );
  fprintf( fp, "<BODY>\n" );
}

  void
create_page_hr( FILE * fp )
{
  fprintf( fp, "<HR>\n" );
}

  void
create_page_table_header( FILE * fp )
{
  fprintf( fp, "<TABLE CELLPADDING=12>\n<TR>\n" );
}

  void
create_page_table_line( FILE * fp )
{
  fprintf( fp, "</TR><TR>\n" );
}

  int
create_page_table_up( FILE * fp, int level )
{
  if (level>0) {
    fprintf( fp,
        "<TD><P ALIGN=CENTER>"
        "<A HREF=\"../index.html\">"
        "<IMG BORDER=0 SRC=\"%s%s/%s\">"
        "<BR>.."
        "</A></P></TD>\n",
        uppath( level ), ICON_DIR, ICON_UP_NAME );
    return(1);
  } else {
    return(0);
  }
}

  void
create_page_table_footer( FILE * fp )
{
  fprintf( fp, "</TR></TABLE>\n" );
}

  void
create_page_dir( FILE * fp, char * dirname, int level )
{
  fprintf( fp,
      "<TD><P ALIGN=CENTER>"
      "<A HREF=\"%s/index.html\">"
      "<IMG BORDER=0 SRC=\"%s%s/%s\">"
      "<BR>%s"
      "</A></P></TD>\n",
      dirname, uppath( level ), ICON_DIR, ICON_FOLDER_NAME, dirname );
}


void
print_page_file( FILE * fp, char *filename, char *level, char *icondir, 
    char *iconname, long long size ) {
  char size_str[80];
  if (size<1024) {
    snprintf( size_str, 80, "%d Bytes", (int) size );
  } else if (size<1024*1024) {
    snprintf( size_str, 80, "%.2f KB", size/(1024.0) );
  } else if (size<1024*1024*1024) {
    snprintf( size_str, 80, "%.2f MB", size/(1024.0*1024.0) );
  } else {
    snprintf( size_str, 80, "%.2f GB", size/(1024.0*1024.0*1024.0) );
  }
  fprintf( fp, 
      "<TD><P>"
      "<A HREF=\"%s\"><IMG SRC=\"%s%s/%s\" BORDER=1><BR>"
      "%s [%s]</A></P></TD>\n",
      filename, level, icondir, iconname, 
      filename, size_str );
}

void
print_gdir_file( FILE * fp, char *filename, char *level, 
    char *icondir, char *maps_iconname, 
    char *log_iconname, long size ) {
  char size_str[80];
  if (size<0) {
    snprintf( size_str, 80, "> 2 GB" );
  } else if (size<1024) {
    snprintf( size_str, 80, "%d Bytes", (int)size );
  } else if (size<1024*1024) {
    snprintf( size_str, 80, "%.2f KB", size/(1024.0) );
  } else if (size<1024*1024*1024) {
    snprintf( size_str, 80, "%.2f MB", size/(1024.0*1024.0) );
  } else {
    snprintf( size_str, 80, "%.2f GB", size/(1024.0*1024.0*1024.0) );
  }
  fprintf( fp, 
      "<TD><P>"
      "<A HREF=\"%s\"><IMG SRC=\"%s%s/%s\" BORDER=1></A>   "
      "<A HREF=\".gdir-%s/overlay.html\">"
      "<IMG ALIGN=TOP WIDTH=50 SRC=\"%s%s/%s\" BORDER=2></A><BR>"
      "<A HREF=\"%s\">%s [%s]</P></TD>\n",
      filename, level, icondir, log_iconname, 
      filename, level, icondir, maps_iconname, 
      filename, filename, size_str );
}

  void
create_page_file( FILE * fp, char * filename, long long file_size,
    char * dirname, int level )
{
  char  call[MAX_NAME_LENGTH];
  char  gfilename[MAX_NAME_LENGTH];
  char  afilename[MAX_NAME_LENGTH];
  char  logname[MAX_NAME_LENGTH];
  char  gdirname[MAX_NAME_LENGTH];
  int   r;

  if (!fnmatch( "*.c", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_C_NAME, file_size );
  } else if (!fnmatch( "*.cpp", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_CPP_NAME, file_size );
  } else if (!fnmatch( "*.h", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_H_NAME, file_size );
  } else if (!fnmatch( "*.moc", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_MOC_NAME, file_size );
  } else if (!fnmatch( "*.java", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_JAVA_NAME, file_size );
  } else if (!fnmatch( "*.o", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOURCE_O_NAME, file_size );
  } else if (!fnmatch( "*.pdf", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_PDF_NAME, file_size );
  } else if (!fnmatch( "*.ps", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_POSTSCRIPT_NAME, file_size );
  } else if (!fnmatch( "*.m2t", filename, FNM_PATHNAME | FNM_CASEFOLD )) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_VIDEO_NAME, file_size );
  } else if ((!fnmatch( "*.jpg", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.png", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.gif", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.ppm", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.xbm", filename, FNM_PATHNAME | FNM_CASEFOLD ))) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_IMAGE_NAME, file_size );
  } else if ((!fnmatch( "*.avi", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.mpg", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.mpeg", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.mov", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.rm", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.wmv", filename, FNM_PATHNAME | FNM_CASEFOLD ))) {
    if (!use_ffmpeg || file_size < MIN_VIDEO_SIZE) {
      print_page_file( fp, filename, uppath( level ), ICON_DIR, 
          ICON_VIDEO_NAME, file_size );
    } else {
      snprintf( gfilename, MAX_NAME_LENGTH, "%s.gif",filename );
      snprintf( afilename, MAX_NAME_LENGTH, "%s/%s/%s",
          dirname, ANIMATED_GIF_DIR, gfilename );
      if (force || !file_exists(afilename)) {
        fprintf( stderr, "# INFO: create animated gif for: %s\n", filename );
        snprintf( call, MAX_NAME_LENGTH,
            "ffmpeg -i '%s/%s' -s %dx%d -t %f -r %f -pix_fmt rgb24 "
            "-loop_output 0 -y '%s' >& /dev/null",
            dirname, filename, ICON_WIDTH, ICON_HEIGHT,
            ANIMATED_GIF_LENGTH_IN_SEC, ANIMATED_GIF_FRAME_RATE, 
            afilename );
        if(system( call ) == -1)
          dgc_error("Failed to run command %s", call);
      }
      print_page_file( fp, filename, "", ANIMATED_GIF_DIR, 
          gfilename, file_size );
    }
  } else if ((!fnmatch( "*.wav", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.mp3", filename, FNM_PATHNAME | FNM_CASEFOLD ))) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_SOUND_NAME, file_size );
  } else if ((!fnmatch( "*.vlf", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.pcap", filename, FNM_PATHNAME | FNM_CASEFOLD ))) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_VELODYNE_NAME, file_size );
  } else if ((!fnmatch( "*.bcf", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.bcf.gz", filename, FNM_PATHNAME | FNM_CASEFOLD )) ) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_BOSCHCAM_NAME, file_size );
  } else if ((!fnmatch( "*.log", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.log.gz", filename, FNM_PATHNAME | FNM_CASEFOLD ))) {
    if (googledir) {
      snprintf( logname, MAX_NAME_LENGTH, "%s/%s", dirname, filename );
      snprintf( gdirname, MAX_NAME_LENGTH, "%s/.gdir-%s", dirname, filename );
      fprintf(stderr, "# INFO: googledir for %s ... ", logname );
      r = create_google_webdir( logname, gdirname, force );
      if (r==1) {
        fprintf( stderr, "skipped\n" );
        print_gdir_file( fp, filename, uppath( level ), ICON_DIR, 
            ICON_GOOGLE_MAPS_NAME, ICON_JUNIOR_NAME, file_size );
      } else if (r==-1) {
        fprintf( stderr, "no pose\n" );
        print_page_file( fp, filename, uppath( level ), ICON_DIR, 
            ICON_JUNIOR_NAME, file_size );
      } else {
        fprintf( stderr, "ok\n" );
        print_gdir_file( fp, filename, uppath( level ), ICON_DIR, 
            ICON_GOOGLE_MAPS_NAME, ICON_JUNIOR_NAME, file_size );
      }
    } else {
      print_page_file( fp, filename, uppath( level ), ICON_DIR, 
          ICON_JUNIOR_NAME, file_size );
    }
  } else if ((!fnmatch( "*.pgr", filename, FNM_PATHNAME | FNM_CASEFOLD )) ||
      (!fnmatch( "*.llf", filename, FNM_PATHNAME | FNM_CASEFOLD )) ) {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_LADYBUG_NAME, file_size );
  } else {
    print_page_file( fp, filename, uppath( level ), ICON_DIR, 
        ICON_DOCUMENT_NAME, file_size );
  }
}

  void
create_page_text( FILE * fp, char * text )
{
  fprintf( fp, "<P>%s</P>\n", text );
}

  void
create_page_footer( FILE * fp, int level __attribute__((unused)) )
{
  fprintf( fp, "</BODY>\n" );
}

  void
printdir( char * dirname, int level )
{
  char               wrong_folder[MAX_NAME_LENGTH];
  char               indexname[MAX_NAME_LENGTH];
  char               animdirname[MAX_NAME_LENGTH];
  char               absname[MAX_NUM_FILES][MAX_NAME_LENGTH];
  struct dirent   ** namelist;
  struct stat64      st;
  int                i, n;
  FILE             * fp;
  int                cnt, filecnt = 0, dircnt = 0;

  snprintf( wrong_folder, MAX_NAME_LENGTH, "*%s", "lost+found" );
  if (!fnmatch( wrong_folder, dirname, 0 )) {
    fprintf( stderr, "# INFO: skip folder: %s\n", dirname );
    return;
  }

  snprintf( wrong_folder, MAX_NAME_LENGTH, "*%s", ICON_DIR );
  if (!fnmatch( wrong_folder, dirname, 0 )) {
    fprintf( stderr, "# INFO: skip folder: %s\n", dirname );
    return;
  }

  snprintf( wrong_folder, MAX_NAME_LENGTH, "*%s", ANIMATED_GIF_DIR );
  if (!fnmatch( wrong_folder, dirname, 0 )) {
    fprintf( stderr, "# INFO: skip folder: %s\n", dirname );
    return;
  }

  n = scandir( dirname, &namelist, 0, alphasort);

  if (n>MAX_NUM_FILES) n = MAX_NUM_FILES;
  for (i=0; i<n; i++) {
    snprintf( absname[i], MAX_NAME_LENGTH,
        "%s/%s", dirname, namelist[i]->d_name );
    stat64( absname[i], &st );
    if ( namelist[i]->d_name[0] != '.' &&
        strcmp( namelist[i]->d_name, "lost+found") ) {
      if ( S_ISDIR( st.st_mode ) ) {
        dircnt++;
      } else if (strcasecmp( namelist[i]->d_name, "index.htm" ) &&
          strcasecmp( namelist[i]->d_name, "index.html" ) &&
          fnmatch( "*.kml", namelist[i]->d_name, FNM_CASEFOLD ) ) {
        filecnt++;
      }
    }
  }

  snprintf( indexname, MAX_NAME_LENGTH,
      "%s/%s", dirname, FLAG_FILE_NAME );
  if ((fp=fopen(indexname,"w"))!=0) {
    fclose(fp);
  }
  snprintf( indexname, MAX_NAME_LENGTH,
      "%s/%s", dirname, "index.html" );
  if ((fp=fopen(indexname,"w"))==0) {
    fprintf( stderr, "ERROR: can't write index file %s !\n", indexname );
    exit(0);
  }

  snprintf( animdirname, MAX_NAME_LENGTH, "%s/%s", 
      dirname, ANIMATED_GIF_DIR );
  if (mkdir( animdirname, 493 ) > 0 ) {
    fprintf( stderr, "ERROR: can't make animated-gif dir %s !\n", 
        animdirname );
    exit(0);
  }

  create_page_header( fp, level );
  create_page_hr( fp );

  /* directories first! */
  if (level>0 || dircnt>0) {
    create_page_text( fp, "DIRECTORIES:" );
    create_page_table_header( fp );
    cnt = create_page_table_up( fp, level );
    for (i=2; i<n; i++) {
      stat64( absname[i], &st );
      if ( S_ISDIR( st.st_mode ) && 
          namelist[i]->d_name[0] != '.' &&
          strcmp( namelist[i]->d_name, "lost+found") ) {
        create_page_dir( fp, namelist[i]->d_name, level );
        if (cnt++>NUM_DIR_ICON_PER_ROW-2) {
          create_page_table_line( fp );
          cnt = 0;
        }
      }
    }
    create_page_table_footer( fp );
    create_page_hr( fp );
  }

  /* then the other files */

  if (filecnt>0) {
    create_page_text( fp, "FILES:" );
    create_page_table_header( fp );
    cnt = 0;
    for (i=2; i<n; i++) {
      stat64( absname[i], &st );
      if ( !S_ISDIR( st.st_mode ) &&
          namelist[i]->d_name[0] != '.' &&
          strcmp( namelist[i]->d_name, "lost+found") &&
          ( strcasecmp( namelist[i]->d_name, "index.htm" ) &&
            strcasecmp( namelist[i]->d_name, "index.html" ) ) ) {
        create_page_file( fp, namelist[i]->d_name, st.st_size, 
            dirname, level );
        if (cnt++>NUM_FILE_ICON_PER_ROW-2) {
          create_page_table_line( fp );
          cnt = 0;
        }
      }
    }
    create_page_table_footer( fp );
    create_page_hr( fp );
  }

  create_page_footer( fp, level );
  fclose(fp);

  /* now recursive! */
  if (recursive) {
    for (i=2; i<n; i++) {
      stat64( absname[i], &st );
      if ( S_ISDIR( st.st_mode ) ) {
        printdir( absname[i], level+1 );
      }
    }
  }

  if ( i>=0 ) {
    for (i=0; i<n; i++) {
      free(namelist[i]);
    }
    free(namelist);
  }
}

  void
print_usage( char * prgname )
{
  fprintf(stderr, "\nusage: %s [options]\n", prgname );
  fprintf(stderr, "  -r, --recursive:        use recursive mode\n" );
  fprintf(stderr, "  -f, --force:            force creating files\n" );
}


  int
main( int argc, char** argv )
{
  int     i;

  for (i=1; i<argc; i++) {
    if (!strcmp(argv[i],"--recursive") || !strcmp(argv[i],"-r") ) {
      recursive = TRUE;
    } else if (!strcmp(argv[i],"--force") || !strcmp(argv[i],"-f") ) {
      force = TRUE;
    } else {
      print_usage(argv[0]);
      exit(1);
    }
  }

  if (mkdir( ICON_DIR, 493 ) > 0 ) {
    fprintf( stderr, "ERROR: can't make icon dir %s !\n", ICON_DIR );
    exit(0);
  } else {
    create_icons();
  }

  printdir( ".", 0 );
  return(0);
}


