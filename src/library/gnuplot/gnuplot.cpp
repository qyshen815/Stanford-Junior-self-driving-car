#include <roadrunner.h>
#include <string>
#include "gnuplot.h"

gnuplot::gnuplot()
{
  char temp[200];
  int i, j;

  columns_ = 1;
  rows_ = 1;

  multiplot_count = (int **)calloc(columns_, sizeof(int *));
  dgc_test_alloc(multiplot_count);
  for(i = 0; i < columns_; i++) {
    multiplot_count[i] = (int *)calloc(rows_, sizeof(int));
    dgc_test_alloc(multiplot_count[i]);
    for(j = 0; j < rows_; j++)
      multiplot_count[i][j] = 0;
  }

  title_set = (int **)calloc(columns_, sizeof(int *));
  dgc_test_alloc(title_set);
  for(i = 0; i < columns_; i++) {
    title_set[i] = (int *)calloc(rows_, sizeof(int));
    dgc_test_alloc(title_set[i]);
    for(j = 0; j < rows_; j++)
      title_set[i][j] = 0;
  }

  sprintf(temp, "set size 1,1\n");
  plotstr_ += temp;
  sprintf(temp, "set origin 0,0\n");
  plotstr_ += temp;
  sprintf(temp, "set multiplot\n");
  plotstr_ += temp;

  current_row = 0;
  current_col = 0;
}

gnuplot::gnuplot(int columns, int rows)
{
  char temp[200];
  int i, j;

  columns_ = columns;
  rows_ = rows;

  multiplot_count = (int **)calloc(columns, sizeof(int *));
  dgc_test_alloc(multiplot_count);
  for(i = 0; i < columns; i++) {
    multiplot_count[i] = (int *)calloc(rows, sizeof(int));
    dgc_test_alloc(multiplot_count[i]);
    for(j = 0; j < rows; j++)
      multiplot_count[i][j] = 0;
  }

  title_set = (int **)calloc(columns, sizeof(int *));
  dgc_test_alloc(title_set);
  for(i = 0; i < columns; i++) {
    title_set[i] = (int *)calloc(rows, sizeof(int));
    dgc_test_alloc(title_set[i]);
    for(j = 0; j < rows; j++)
      title_set[i][j] = 0;
  }

  sprintf(temp, "set size 1,1\n");
  plotstr_ += temp;
  sprintf(temp, "set origin 0,0\n");
  plotstr_ += temp;
  sprintf(temp, "set multiplot\n");
  plotstr_ += temp;

  current_row = 0;
  current_col = 0;
}

gnuplot::~gnuplot()
{
  int i;

  for(i = 0; i < columns_; i++) {
    free(multiplot_count[i]);
    free(title_set[i]);
  }
  free(multiplot_count);
  free(title_set);
}

void gnuplot::start_subplot(int column, int row)
{
  char temp[200];
  
  if(multiplot_count[current_col][current_row] > 0)
    finish_subplot();

  if((column != current_col || row != current_row) &&
     multiplot_count[column][row] > 0)
    dgc_die("Error: can't switch back to previously drawn subgraph.\n");
  
  current_col = column;
  current_row = row;
  sprintf(temp, "set size %f,%f\n", 1 / (double)columns_, 1 / (double)rows_);
  plotstr_ += temp;
  sprintf(temp, "set origin %f,%f\n", column / (double)columns_, 
	  row / (double)rows_);
  plotstr_ += temp;
}

char *escape_string(char *str)
{
  static char output[1000];
  int i, mark;

  output[0] = '\0';
  mark = 0;
  for(i = 0; i < (int)strlen(str); i++) {
    if(str[i] == '_') {
      output[mark] = '\\';
      mark++;
    } 
    output[mark] = str[i];
    mark++;
  }
  output[mark] = '\0';
  return output;
}

void gnuplot::xy_plot(char *filename, int col1, int col2, char *title,
		      char *withstyle)
{
  char temp[200];

  if(multiplot_count[current_col][current_row] == 0) {
    if(xlabel_.length() > 0) {
      sprintf(temp, "set xlabel \"%s\"\n", xlabel_.c_str());
      plotstr_ += temp;
    }
    if(ylabel_.length() > 0) {
      sprintf(temp, "set ylabel \"%s\"\n", ylabel_.c_str());
      plotstr_ += temp;
    }
    if(!title_set[current_col][current_row]) {
      sprintf(temp, "set title \"\"\n");
      plotstr_ += temp;
    }

    if(title != NULL)
      sprintf(temp, "plot \"%s\" using %d:%d title '%s' with %s", 
	      filename, col1, col2, escape_string(title), 
	      withstyle != NULL ? withstyle : "lines");
    else
      sprintf(temp, "plot \"%s\" using %d:%d with %s", 
	      filename, col1, col2,
	      withstyle != NULL ? withstyle : "lines");
  }
  else {
    if(title != NULL)
      sprintf(temp, ", \"%s\" using %d:%d title '%s' with %s ",
	      filename, col1, col2, escape_string(title),
	      withstyle != NULL ? withstyle : "lines");
    else
      sprintf(temp, ", \"%s\" using %d:%d with %s ",
	      filename, col1, col2,
	      withstyle != NULL ? withstyle : "lines");
  }
  plotstr_ += temp;

  (multiplot_count[current_col][current_row])++;
}

void gnuplot::set_pointsize(double mult)
{
  char temp[200];

  sprintf(temp, "set pointsize %f\n", mult);
  plotstr_ += temp;  
}

void gnuplot::line_plot(double m, double b, char *title)
{
  char temp[200];

  if(multiplot_count[current_col][current_row] == 0) 
    dgc_die("Error: can't plot line as first layer of a plot.\n");
  else {
    if(title != NULL)
      sprintf(temp, ", %f+%f*x title '%s' ", b, m, title);
    else
      sprintf(temp, ", %f+%f*x ", b, m);
  }
  plotstr_ += temp;

  (multiplot_count[current_col][current_row])++;
}

void gnuplot::plot_function(char *f, char *title)
{
  char temp[200];

  if(multiplot_count[current_col][current_row] == 0) 
    dgc_die("Error: can't plot function as first layer of a plot.\n");
  else {
    if(title != NULL)
      sprintf(temp, ", %s title '%s' ", f, title);
    else
      sprintf(temp, ", %s ", f);
  }
  plotstr_ += temp;

  (multiplot_count[current_col][current_row])++;

}

void gnuplot::finish_subplot(void)
{
  plotstr_ += '\n';
}

void gnuplot::set_title(char *title)
{
  char temp[200];

  title_set[current_col][current_row] = 1;
  sprintf(temp, "set title \"%s\"\n", title);
  plotstr_ += temp;
}

void gnuplot::render(char *filename)
{
  FILE *fp;

  fprintf(stderr, "Plotting %s\n", filename);

  if(multiplot_count[current_col][current_row] > 0)
    finish_subplot();

  fp = fopen("/tmp/script.txt", "w");
  if(fp == NULL)
    dgc_die("Error: could not open script.txt for writing.\n");

  if(strcmp(dgc_file_extension(filename), ".gif") == 0) 
    fprintf(fp, "set terminal gif\n");
  else
    fprintf(fp, "set terminal postscript enhanced color solid\n");
  fprintf(fp, "set output \"%s\"\n", filename);
  
  fprintf(fp, "%s", plotstr_.c_str());

  fprintf(fp, "unset multiplot\n");

  fclose(fp);
  
  if(system("gnuplot /tmp/script.txt") == -1)
    dgc_error("spawning gnuplot failed");
  //  system("rm /tmp/script.txt");
}

