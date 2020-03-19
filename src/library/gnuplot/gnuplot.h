#ifndef DGC_GNUPLOT_H
#define DGC_GNUPLOT_H

#include <roadrunner.h>
#include <string>

class gnuplot {
public:
  gnuplot();
  gnuplot(int columns, int rows);
  ~gnuplot();

  void set_title(char *title);
  void set_xlabel(char *label) { xlabel_ = label; }
  void set_ylabel(char *label) { ylabel_ = label; }

  void start_subplot(int column, int row);
  void xy_plot(char *filename, int col1, int col2, char *title = NULL, 
	       char *with_style = NULL);
  void line_plot(double m, double b, char *title = NULL);
  void plot_function(char *f, char *title = NULL);
  
  void set_pointsize(double mult);

  void render(char *filename);

private:
  void finish_subplot(void);

  int columns_, rows_;
  int current_row, current_col;
  int **multiplot_count, **title_set;
  std::string xlabel_, ylabel_, plotstr_;
};

#endif
