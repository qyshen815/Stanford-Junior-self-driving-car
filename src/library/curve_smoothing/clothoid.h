#ifndef CURVE_SMOOTHING_CLOTHOID_H_
#define CURVE_SMOOTHING_CLOTHOID_H_

#ifndef ORDER
  #define ORDER 12
#endif

    /* Possible values of the "ty" field. */
#define SPIRO_CORNER    'v'
#define SPIRO_G4    'o'
#define SPIRO_G2    'c'
#define SPIRO_LEFT    '['
#define SPIRO_RIGHT   ']'

    /* For a closed contour add an extra cp with a ty set to */
#define SPIRO_END   'z'
    /* For an open contour the first cp must have a ty set to*/
#define SPIRO_OPEN_CONTOUR  '{'
    /* For an open contour the last cp must have a ty set to */
#define SPIRO_END_OPEN_CONTOUR  '}'

class BezierContext {
public:
    virtual void moveto(double x, double y, int is_open) = 0;
    virtual void lineto(double x, double y) = 0;
    virtual void curveto(double x1, double y1, double x2, double y2, double x3, double y3) = 0;
};

class Clothoid {

public:
  Clothoid() {
     n = 4;
    #if 0
     n = 1024;
    #endif
  }
  virtual ~Clothoid() {}

  int n;

 typedef struct {
    double x;
    double y;
    char ty;
} spiro_cp;

typedef struct {
  double a[11]; /* band-diagonal matrix */
  double al[5]; /* lower part of band-diagonal decomposition */
} bandmat;

typedef struct spiro_seg_s {
  double x;
  double y;
  char ty;
  double bend_th;
  double ks[4];
  double seg_ch;
  double seg_th;
  double l;
} spiro_seg;

void SpiroCPsToBezier(spiro_cp *spiros, int n, int isclosed, BezierContext *bc);
void TaggedSpiroCPsToBezier(spiro_cp *spiros, BezierContext *bc);

private:
  spiro_seg* run_spiro(const spiro_cp *src, int n);
  void free_spiro(spiro_seg* s);
  void spiro_to_bpath(const spiro_seg* s, int n, BezierContext* bc);
  double get_knot_th(const spiro_seg *s, int i);
  void integrate_spiro(const double ks[4], double xy[2]); // TODO: temporarily public

  double compute_ends(const double ks[4], double ends[2][4], double seg_ch);
  void compute_pderivs(const spiro_seg *s, double ends[2][4], double derivs[4][2][4], int jinc);
  double mod_2pi(double th);
  void bandec11(bandmat *m, int *perm, int n);
  void banbks11(const bandmat *m, const int *perm, double *v, int n);
  int compute_jinc(char ty0, char ty1);
  int count_vec(const spiro_seg *s, int nseg);
  void add_mat_line(bandmat *m, double *v, double derivs[4], double x, double y, int j, int jj, int jinc, int nmat);
  double spiro_iter(spiro_seg *s, bandmat *m, int *perm, double *v, int n);
  void spiro_seg_to_bpath(const double ks[4], double x0, double y0, double x1, double y1, BezierContext *bc, int depth);
  spiro_seg* setup_path(const spiro_cp *src, int n);
  int solve_spiro(spiro_seg *s, int nseg);
};

#endif // CURVE_SMOOTHING_CLOTHOID_H_
