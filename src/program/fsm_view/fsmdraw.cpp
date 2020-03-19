#include <roadrunner.h>
#include <gl_support.h>
#include "fsmdraw.h"

using namespace dgc;

static inline void draw_arrowhead(double x, double y, double angle)
{
  double ct, st, l = 0.15, l2 = 0.07;

  ct = cos(angle);
  st = sin(angle);
  glPushMatrix();
  glTranslatef(l * ct, l * st, 0);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
  glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
  glEnd();
  glPopMatrix();
}

void draw_ellipse(double x, double y, double width, double height)
{
  int i;
  double theta;

  glPushMatrix();
  glTranslatef(x, y, 0);
  glScalef(width / 2, height / 2, 1);
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 50; i++) {
    theta = 2 * M_PI * i / 50.0;
    glVertex2f(cos(theta), sin(theta));
  }
  glEnd();
  glPopMatrix();
}

void fill_ellipse(double x, double y, double width, double height)
{
  int i;
  double theta;

  glPushMatrix();
  glTranslatef(x, y, 0);
  glScalef(width / 2, height / 2, 1);
  glBegin(GL_POLYGON);
  for(i = 0; i < 50; i++) {
    theta = 2 * M_PI * i / 50.0;
    glVertex2f(cos(theta), sin(theta));
  }
  glEnd();
  glPopMatrix();
}

inline int factorial(int x)
{
  int i, r = 1;

  for(i = 2; i <= x; i++)
    r *= i;
  return r;
}

inline double bernstein(int n, int i, double u)
{
  return factorial(n) / (factorial(n - i) * factorial(i)) * pow(u, i) * 
    pow(1 - u, n - i);
}

void draw_edge(fsm_edge_p edge)
{
  int n = edge->num_points;
  double u, x, y, k;
  int i, j;

  glBegin(GL_LINE_STRIP);
  for(i = 0; i <= 30; i++) {
    u = i / (double)30.0;
    
    x = 0; y = 0;
    for(j = 0; j < edge->num_points; j++) {
      k = bernstein(n - 1, j, u);
      x += k * edge->point[j * 3];
      y += k * edge->point[j * 3 + 1];
    }
    glVertex2f(x, y);
  }
  glEnd();

  draw_arrowhead(edge->point[(n - 1) * 3], edge->point[(n - 1) * 3 + 1],
		 atan2(edge->point[(n - 1) * 3 + 1] - 
		       edge->point[(n - 2) * 3 + 1],
		       edge->point[(n - 1) * 3 + 0] - 
		       edge->point[(n - 2) * 3 + 0]));
}

void draw_edge_simple(fsm_edge_p edge)
{
  int i;
  int n = edge->num_points;

  glBegin(GL_LINE_STRIP);
  for(i = 0; i < n; i++) 
    glVertex2f(edge->point[i * 3], edge->point[i * 3 + 1]);
  glEnd();

  glColor3f(1, 1, 0);
  for(i = 0; i < n; i++) 
    draw_circle(edge->point[i * 3], edge->point[i * 3 + 1], .05);
}

void fsm_draw_graph(fsm_graph_p graph, double x, double y, double size,
		    int highlight_node)
{
  int i;
  
  glPushMatrix();
  glScalef(size / (graph->width / 2.0), size / (graph->width / 2.0), 1.0);

  glTranslatef(x, y, 0);

  glTranslatef(-graph->width / 2.0, -graph->height / 2.0, 0.0);

#ifdef blah
  if(highlight_node != -1) {
        glLineWidth(5);
    glColor3f(1, 1, 0);
    glPushMatrix();

    glTranslatef(graph->node[highlight_node].x, 
		 graph->node[highlight_node].y, 0);

    fill_ellipse(0, 0, graph->node[highlight_node].width * 1.05, 
		 graph->node[highlight_node].height +
		 graph->node[highlight_node].width * 0.05);

    //    draw_ellipse(0, 0, graph->node[highlight_node].width, 
    //		 graph->node[highlight_node].height);

    /*
    glBegin(GL_POLYGON);
    glVertex2f(-graph->node[highlight_node].width / 2,
	       -graph->node[highlight_node].height / 2);
    glVertex2f(graph->node[highlight_node].width / 2,
	       -graph->node[highlight_node].height / 2);
    glVertex2f(graph->node[highlight_node].width / 2,
	       graph->node[highlight_node].height / 2);
    glVertex2f(-graph->node[highlight_node].width / 2,
	       graph->node[highlight_node].height / 2);
	       glEnd(); */

    glLineWidth(1);

    glPopMatrix();
  }
#endif

  for(i = 0; i < (int)graph->node.size(); i++) {
    glPushMatrix();
    glTranslatef(graph->node[i].x, graph->node[i].y, 0);
    
    if(graph->node[i].num == highlight_node) {
      glColor3f(1, 1, 0);
      fill_ellipse(0, 0, graph->node[i].width, graph->node[i].height);
      draw_ellipse(0, 0, graph->node[i].width, graph->node[i].height);
      glColor3f(0, 0, 0);
    }
    else {
      glColor3f(0.7, 0.7, 0.7);
      fill_ellipse(0, 0, graph->node[i].width, graph->node[i].height);
      draw_ellipse(0, 0, graph->node[i].width, graph->node[i].height);
      glColor3f(0, 0, 0);
    }
    render_stroke_text_centered_2D(0, 0, GLUT_STROKE_ROMAN,
				   0.14, graph->node[i].label);
    glPopMatrix();
  }

  glColor3f(1, 0.6, 0);
  //  glColor3f(0, 0, 1);
  //      glColor3f(0.5, 0.5, 1);
  //  glColor3f(1, 1, 1);
  for(i = 0; i < (int)graph->edge.size(); i++)
    draw_edge(&graph->edge[i]);

  glPopMatrix();
}

fsm_graph_p fsm_read_graph_from_file(FILE *fp)
{
  char line[1000], *err, *line2;
  fsm_graph_p graph;
  fsm_node_t n;
  fsm_edge_t ne;
  int i;

  graph = new fsm_graph_t;	
  do {				
    err = fgets(line, 1000, fp);
    if(err != NULL) {		
      if(strncmp(line, "graph", 5) == 0) 
	sscanf(line, "%*s %*f %lf %lf", &graph->width, &graph->height);
      else if(strncmp(line, "node", 4) == 0) {
	line2 = dgc_next_word(line);
	sscanf(line2, "%d %lf %lf %lf %lf %s", &n.num, &n.x, &n.y, &n.width, 
	       &n.height, n.label);
	graph->node.push_back(n);
      }
      else if(strncmp(line, "edge", 4) == 0) {
	line2 = dgc_next_n_words(line, 3);

	sscanf(line2, "%d", &ne.num_points);
	ne.point = (float *)calloc(ne.num_points * 3, sizeof(float));

	line2 = dgc_next_word(line2);

	for(i = 0; i < ne.num_points; i++) {
	  sscanf(line2, "%f %f", &ne.point[i * 3 + 0], &ne.point[i * 3 + 1]);
	  ne.point[i * 3 + 2] = 0;
	  
	  line2 = dgc_next_n_words(line2, 2);
	}
	graph->edge.push_back(ne);
      }
    }
  } while(err != NULL);

  return graph;
}

fsm_graph_p fsm_read_graph_from_memory(char *str)
{
  FILE *fp;

  fp = tmpfile();
  fprintf(fp, "%s", str);
  rewind(fp);
  return fsm_read_graph_from_file(fp);
}


